#include "main.h"
#include "tle5014.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

TIM_TypeDef g_tim4; RCC_TypeDef g_rcc; GPIO_TypeDef g_gpiob;
static uint32_t g_tick_ms; static GPIO_PinState g_pin = GPIO_PIN_SET;
uint32_t HAL_GetTick(void){ return g_tick_ms; }
uint32_t HAL_RCC_GetPCLK1Freq(void){ return 36000000U; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin){ (void)p;(void)pin; return g_pin; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i){ (void)p;(void)i; }
void HAL_NVIC_DisableIRQ(int x){(void)x;}
void HAL_NVIC_EnableIRQ(int x){(void)x;}
void HAL_NVIC_SetPriority(int x, uint32_t a, uint32_t b){(void)x;(void)a;(void)b;}

/* CRC độc lập: chia đa thức trên chuỗi bit, seed 0101b */
static uint8_t crc_ref(const uint8_t *n, int cnt){
    unsigned reg = 0x5; 
    for(int i=0;i<cnt;i++) for(int b=3;b>=0;b--){
        unsigned in = (n[i]>>b)&1; unsigned top = (reg>>3)&1;
        reg = (reg<<1)&0xF; if(top^in) reg ^= 0xD;
    }
    return reg;
}

static double t_us = 1000.0;      /* thời gian mô phỏng */
static double ut_us = 3.0;         /* UT thực của cảm biến (có sai số clock) */
static double jitter_us = 0.0;
static int sync_like_pauses = 0;

// Cạnh xuống giả lập (ISR) -> gọi TLE5014_EdgeIRQ() để mô phỏng ngắt
static void edge(void){
    double j = ((rand()/(double)RAND_MAX)*2.0-1.0)*jitter_us;
    g_tim4.CNT = (uint16_t)(uint32_t)((t_us + j)*4.0);   /* 4 tick/us */
    g_tick_ms = (uint32_t)(t_us/1000.0);
    TLE5014_EdgeIRQ();
}


/* Gửi 1 frame. Cạnh bắt đầu sync đã được phát từ trước (cạnh cuối pause của frame trước). */
static void send_frame(uint16_t angle12, uint8_t rc, uint8_t status, int corrupt_crc, int drop_edge_idx, int glitch){
    uint8_t n[8];
    n[0]=status&0xF; n[1]=(angle12>>8)&0xF; n[2]=(angle12>>4)&0xF; n[3]=angle12&0xF;
    n[4]=(rc>>4)&0xF; n[5]=rc&0xF; n[6]=(n[1]^0xF)&0xF; n[7]=crc_ref(&n[1],6);
    if(corrupt_crc) n[7]^=0x3;
    double periods[10]; periods[0]=56;
    int sum=56; for(int i=0;i<8;i++){ periods[1+i]=12+n[i]; sum+=12+n[i]; }
    periods[9]=282-sum;
    if(periods[9]>=56*0.9 && periods[9]<=56*1.1) sync_like_pauses++;
    for(int i=0;i<10;i++){
        t_us += periods[i]*ut_us;
        if(i!=drop_edge_idx) edge();
        if(glitch && i==4){ t_us += 0.8; edge(); t_us -= 0.8; }
    }
}

static TLE5014_Handle_t h;
static int fails=0;
#define CHECK(c,msg) do{ if(!(c)){ printf("  FAIL: %s\n",msg); fails++; } }while(0)

static void run_normal(int frames, double ut, double jit, const char *name){
    ut_us=ut; jitter_us=jit; srand(1);
    TLE5014_Init(&h,NULL);
    t_us=1000; 
    sync_like_pauses=0; 
    g_pin=GPIO_PIN_SET;
    edge();                                   /* cạnh đầu tiên */
    int ok=0, bad=0; uint8_t rc=7;

    // Gửi nhiều frame liên tiếp, Task() chạy sau mỗi frame, kiểm tra góc hợp lệ
    for(int i=0;i<frames;i++){
        uint16_t ang = 1 + (rand()%4088);
        send_frame(ang, rc, 0, 0, -1, 0);
        TLE5014_Task(&h);                      /* Task chạy sau mỗi frame (nhanh) */
        TLE5014_Angle_t a; 
        TLE5014_Status_t st = TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
        if(st==TLE5014_OK && a.raw_angle==ang && a.rolling_counter==rc) ok++; 
        else bad++;
        rc++;
    }
    printf("\n[%s] \nframes=%d ok=%d bad=%d sync_like_pause=%d | cap=%u resync=%u nibErr=%u crcErr=%u rollErr=%u overrun=%u\n",
        name, frames, ok, bad, sync_like_pauses, tle5014_sent_debug.frames_captured, tle5014_sent_debug.resyncs,
        tle5014_sent_debug.nibble_errors, tle5014_sent_debug.crc_errors, tle5014_sent_debug.rolling_errors, tle5014_sent_debug.overruns);
    /* frame đầu tiên không thể bắt được (chưa có cạnh tham chiếu đầy đủ) -> cho phép thiếu 1-2 frame đầu */
    CHECK(bad<=2, "qua nhieu frame loi");
}

int main(void){
    g_rcc.CFGR = 0x400;   /* APB1 chia 2 -> timer = 72 MHz */
    run_normal(3000, 3.0,  0.0, "UT 3.00us, khong jitter ");
    run_normal(3000, 3.15, 0.0, "UT +5% clock          ");
    run_normal(3000, 2.85, 0.0, "UT -5% clock          ");
    run_normal(3000, 3.0,  0.4, "jitter (cong/tru) 0.4us          ");
    run_normal(3000, 3.15, 0.4, "UT +5% & jitter (cong/tru) 0.4us ");

    /* ---- Kịch bản lỗi ---- */
    printf("\n== Kich ban loi ==\n");
    ut_us=3.0; jitter_us=0; srand(2);
    TLE5014_Init(&h,NULL); t_us=1000; g_pin=GPIO_PIN_SET; edge();
    uint8_t rc=0; TLE5014_Angle_t a; TLE5014_Status_t st;
    for(int i=0;i<5;i++){ send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);} 
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    CHECK(st==TLE5014_OK,"khoi dong OK"); printf("goc 2048 -> %.3f deg (status %d)\n", a.angle_degrees, st);

    /* 1) CRC sai 1 frame: phải ghi nhận CRC_ERROR nhưng chưa mất hiệu lực (debounce) */
    send_frame(2048, rc++, 0,1,-1,0); TLE5014_Task(&h);
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("1 frame CRC sai: status handle=%d, ReadAngle=%d, crc_err=%u\n", h.channel1_status, st, tle5014_sent_debug.crc_errors);
    CHECK(h.channel1_status==TLE5014_CRC_ERROR,"CRC_ERROR");
    CHECK(st==TLE5014_OK,"1 frame loi chua mat hieu luc"); CHECK(h.channel1_valid==1,"goc hop le");
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    CHECK(TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a)==TLE5014_OK,"hoi phuc sau 1 frame loi");

    /* 2) 3 frame CRC sai liên tiếp -> mất hiệu lực */
    for(int i=0;i<3;i++){ send_frame(2048, rc++, 0,1,-1,0); TLE5014_Task(&h);} 
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("3 frame CRC sai lien tiep: ReadAngle=%d valid=%d\n", st, h.channel1_valid);
    CHECK(st==TLE5014_CRC_ERROR && h.channel1_valid==0,"mat hieu luc sau 3 loi lien tiep");
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    CHECK(TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a)==TLE5014_OK,"hoi phuc");

    /* 3) rolling counter bị lặp (frame lặp lại) */
    send_frame(2048, (uint8_t)(rc-1), 0,0,-1,0); TLE5014_Task(&h);   /* lặp lại rc của frame trước */
    printf("rolling lap: status=%d (mong doi %d)\n", h.channel1_status, TLE5014_ROLLING_ERROR);
    CHECK(h.channel1_status==TLE5014_ROLLING_ERROR,"phat hien rolling lap");
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);

    /* 4) frame bị mất 1 cạnh trong ISR -> frame đó bị bỏ, frame sau rolling nhảy 2 -> ROLLING_ERROR */
    send_frame(2048, rc++, 0,0,3,0); TLE5014_Task(&h);   /* mất 1 cạnh giữa frame (rc đã tăng) */
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    printf("mat canh: status sau frame ke = %d (mong doi ROLLING_ERROR=%d hoac OK neu da resync)\n", h.channel1_status, TLE5014_ROLLING_ERROR);
    CHECK(h.channel1_status==TLE5014_ROLLING_ERROR || h.channel1_status==TLE5014_OK,"hop le");
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    CHECK(TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a)==TLE5014_OK,"hoi phuc sau mat canh");

    /* 5) glitch thừa 1 cạnh */
    send_frame(2048, rc++, 0,0,-1,1); TLE5014_Task(&h);
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    printf("glitch thua canh: ReadAngle=%d nibErr=%u\n", TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a), tle5014_sent_debug.nibble_errors);

    /* 6) cảm biến báo lỗi: status bit0 */
    send_frame(2048, rc++, 1,0,-1,0); TLE5014_Task(&h);
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("status bit0: ReadAngle=%d\n", st);
    CHECK(st==TLE5014_SENSOR_ERROR,"SENSOR_ERROR tuc thi");
    /* mã 4091 */
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);
    send_frame(4091, rc++, 0,0,-1,0); TLE5014_Task(&h);
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("ma 4091: ReadAngle=%d\n", st);
    CHECK(st==TLE5014_SENSOR_ERROR,"4091 -> SENSOR_ERROR");
    send_frame(2048, rc++, 0,0,-1,0); TLE5014_Task(&h);

    /* 7) mất tín hiệu */
    t_us += 6000; g_tick_ms=(uint32_t)(t_us/1000); g_pin=GPIO_PIN_SET; TLE5014_Task(&h);
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("mat tin hieu (day o muc cao): %d\n", st); CHECK(st==TLE5014_NO_SIGNAL,"NO_SIGNAL");
    g_pin=GPIO_PIN_RESET; TLE5014_Task(&h);
    st=TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
    printf("day ket thap: %d\n", st); CHECK(st==TLE5014_STUCK_LOW,"STUCK_LOW");
    /* có tín hiệu lại sau 30 ms (timer 16 bit đã quay vòng): frame đầu có thể mất, sau đó phải hồi phục */
    g_pin=GPIO_PIN_SET; t_us += 30000; edge();
    int rec=0; for(int i=0;i<6;i++){ send_frame(1000, rc++, 0,0,-1,0); TLE5014_Task(&h);
        if(TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a)==TLE5014_OK){ rec=i+1; break; } }
    printf("hoi phuc sau mat tin hieu o frame thu %d\n", rec); CHECK(rec>0,"hoi phuc");

    /* 8) Task() gọi chậm: chỉ gọi mỗi 5 frame -> kiểm tra vòng đệm, không báo rolling lỗi giả */
    printf("\n== Task cham (goi moi 5 frame, ~4 ms) ==\n");
    TLE5014_Init(&h,NULL); t_us=1000; g_tick_ms=1; edge(); rc=0; int rolling_bad=0;
    for(int i=0;i<2000;i++){ send_frame(1+rand()%4088, rc++, 0,0,-1,0);
        if(i%5==4){ TLE5014_Task(&h); if(h.channel1_status==TLE5014_ROLLING_ERROR||h.channel1_status==TLE5014_CRC_ERROR) rolling_bad++; } }
    printf("rolling/crc bao sai=%d overruns=%u frames_ok=%u\n", rolling_bad, tle5014_sent_debug.overruns, tle5014_sent_debug.frames_ok);
    CHECK(rolling_bad==0 && tle5014_sent_debug.overruns==0,"Task cham van dung");

    /* 9) Task rất chậm: mỗi 12 frame (>8 ô đệm) -> overrun được đếm */
    TLE5014_Init(&h,NULL); t_us=1000; g_tick_ms=1; edge(); rc=0;
    for(int i=0;i<1200;i++){ send_frame(1+rand()%4088, rc++, 0,0,-1,0); if(i%12==11) TLE5014_Task(&h); }
    printf("Task rat cham: overruns=%u (mong doi > 0), frames_ok=%u\n", tle5014_sent_debug.overruns, tle5014_sent_debug.frames_ok);
    CHECK(tle5014_sent_debug.overruns>0,"overrun duoc dem");

    /* 10) quy đổi góc */
    TLE5014_Init(&h,NULL); t_us=1000; g_tick_ms=1; edge(); rc=0;
    uint16_t tests[4]={1,2048,4087,4088}; 
    for(int k=0;k<4;k++){ send_frame(tests[k], rc++, 0,0,-1,0); TLE5014_Task(&h); TLE5014_ReadAngle(TLE5014_CHANNEL_1,&a);
        printf("raw %4u -> %.3f deg counts=%u (status %d)\n", tests[k], a.angle_degrees, a.angle_counts, h.channel1_status); }

    printf("\n%s (fails=%d)\n", fails? "CO LOI":"TAT CA DAT", fails);
    return fails?1:0;
}