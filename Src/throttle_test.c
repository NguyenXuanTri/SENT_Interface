#include "throttle.h"
#include <stdio.h>
#include <math.h>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL: %s (dòng %d)\n", msg, __LINE__); fails++; } } while (0)
#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static Throttle_t thr;

static void calib(float closed, float open)
{
    Throttle_Init(&thr, NULL);
    CHECK(Throttle_LoadCalibration(&thr, closed, open) == THR_CAL_OK, "hieu chinh OK");
}

int main(void)
{
    /* 1) Chưa hiệu chỉnh */
    Throttle_Init(&thr, NULL);
    Throttle_Update(&thr, 100.0f, 1U);
    CHECK(thr.state == THR_NOT_CALIBRATED && thr.valid == 0U, "chua hieu chinh -> khong hop le");

    /* 2) Hành trình thường: đóng 100°, mở hết 150° (span 50, chiều +) */
    calib(100.0f, 150.0f);
    CHECK(thr.direction == 1 && NEAR(thr.span_deg, 50.0f, 1e-4f), "chieu + / span 50");
    Throttle_Update(&thr, 100.0f, 1U);
    CHECK(thr.valid && NEAR(thr.raw_percent, 0.0f, 1e-3f) && thr.position_permille == 0U, "dong = 0%");
    Throttle_Update(&thr, 125.0f, 1U);
    CHECK(thr.valid && NEAR(thr.raw_percent, 50.0f, 1e-3f) && NEAR(thr.position_percent, 50.0f, 1e-3f)
          && thr.position_permille == 500U, "giua qua trinh = 50%");
    Throttle_Update(&thr, 150.0f, 1U);
    CHECK(thr.valid && NEAR(thr.position_percent, 100.0f, 1e-3f) && thr.position_permille == 1000U, "mo het = 100%");

    /* 3) Vùng chết 2% mỗi đầu */
    Throttle_Update(&thr, 100.0f + 50.0f * 0.02f, 1U);                /* raw 2% */
    CHECK(NEAR(thr.position_percent, 0.0f, 1e-2f), "raw 2% -> 0% (het vung chet)");
    Throttle_Update(&thr, 100.0f + 50.0f * 0.01f, 1U);                /* raw 1% */
    CHECK(thr.valid && thr.position_percent == 0.0f, "raw 1% -> 0% (trong vung chet)");
    Throttle_Update(&thr, 100.0f + 50.0f * 0.98f, 1U);                /* raw 98% */
    CHECK(NEAR(thr.position_percent, 100.0f, 1e-2f), "raw 98% -> 100%");
    Throttle_Update(&thr, 100.0f + 50.0f * 0.99f, 1U);
    CHECK(thr.position_percent == 100.0f, "raw 99% -> 100%");

    /* 4) Dung sai ngoài 2 đầu: ±5° */
    Throttle_Update(&thr, 97.0f, 1U);                                 /* travel -3: trong dung sai */
    CHECK(thr.valid && thr.position_percent == 0.0f && thr.state == THR_READY, "thap hon dong 3° -> kep 0%, hop le");
    Throttle_Update(&thr, 154.0f, 1U);                                /* travel 54 <= 55 */
    CHECK(thr.valid && thr.position_percent == 100.0f, "vuot mo 4° -> kep 100%, hop le");
    Throttle_Update(&thr, 94.0f, 1U);                                 /* travel -6 */
    CHECK(!thr.valid && thr.state == THR_OUT_OF_RANGE && thr.position_permille == 0U, "thap hon dong 6° -> ngoai hanh trinh");
    Throttle_Update(&thr, 156.0f, 1U);                                /* travel 56 */
    CHECK(!thr.valid && thr.state == THR_OUT_OF_RANGE, "vuot mo 6° -> ngoai hanh trinh");
    Throttle_Update(&thr, 300.0f, 1U);
    CHECK(!thr.valid && thr.state == THR_OUT_OF_RANGE, "goc xa hanh trinh -> ngoai hanh trinh");

    /* 5) Hành trình đi qua mốc 0°/360°: đóng 350°, mở 40° (span 50, chiều +) */
    calib(350.0f, 40.0f);
    CHECK(thr.direction == 1 && NEAR(thr.span_deg, 50.0f, 1e-3f), "qua moc 0: chieu + span 50");
    Throttle_Update(&thr, 350.0f, 1U);  CHECK(thr.valid && thr.position_permille == 0U, "350° = 0%");
    Throttle_Update(&thr, 15.0f, 1U);   CHECK(thr.valid && NEAR(thr.raw_percent, 50.0f, 1e-3f), "15° = 50%");
    Throttle_Update(&thr, 40.0f, 1U);   CHECK(thr.valid && thr.position_permille == 1000U, "40° = 100%");
    Throttle_Update(&thr, 0.0f, 1U);    CHECK(thr.valid && NEAR(thr.raw_percent, 20.0f, 1e-3f), "0° = 20%");

    /* 6) Chiều ngược: đóng 40°, mở 350° (góc giảm khi mở) */
    calib(40.0f, 350.0f);
    CHECK(thr.direction == -1 && NEAR(thr.span_deg, 50.0f, 1e-3f), "chieu nguoc nhan dung");
    Throttle_Update(&thr, 40.0f, 1U);   CHECK(thr.valid && thr.position_permille == 0U, "40° = 0%");
    Throttle_Update(&thr, 15.0f, 1U);   CHECK(thr.valid && NEAR(thr.raw_percent, 50.0f, 1e-3f), "15° = 50% (chieu nguoc)");
    Throttle_Update(&thr, 350.0f, 1U);  CHECK(thr.valid && thr.position_permille == 1000U, "350° = 100% (chieu nguoc)");

    /* 7) Quét liên tục qua mốc 0°: vị trí phải tăng đơn điệu */
    calib(330.0f, 60.0f);               /* span 90 */
    {
        float prev = -1.0f; int mono = 1; int count = 0;
        for (int i = 0; i <= 900; i++)
        {
            float ang = fmodf(330.0f + 0.1f * (float)i, 360.0f);
            Throttle_Update(&thr, ang, 1U);
            if (!thr.valid || thr.raw_percent < prev - 1e-4f) mono = 0;
            prev = thr.raw_percent; count++;
        }
        CHECK(mono && count == 901, "quet 330°->60° tang don dieu, luon hop le");
        CHECK(NEAR(thr.raw_percent, 100.0f, 0.05f), "cuoi quet = 100%");
    }

    /* 8) Góc đầu vào không hợp lệ */
    calib(100.0f, 150.0f);
    Throttle_Update(&thr, 125.0f, 1U);
    Throttle_Update(&thr, 125.0f, 0U);
    CHECK(!thr.valid && thr.state == THR_INPUT_INVALID && thr.position_permille == 0U, "driver bao loi -> vi tri 0, khong hop le");
    Throttle_Update(&thr, NAN, 1U);
    CHECK(!thr.valid && thr.state == THR_INPUT_INVALID, "NaN -> khong hop le");
    Throttle_Update(&thr, 125.0f, 1U);
    CHECK(thr.valid && thr.state == THR_READY, "hoi phuc khi goc hop le tro lai");

    /* 9) Kiểm tra hiệu chỉnh */
    Throttle_Init(&thr, NULL);
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_NOT_READY, "thieu 2 diem");
    Throttle_SetClosed(&thr, 100.0f);
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_NOT_READY, "thieu diem mo");
    Throttle_SetOpen(&thr, 105.0f);
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_SPAN_TOO_SMALL, "hanh trinh 5° qua nho");
    Throttle_SetOpen(&thr, 230.0f);
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_SPAN_TOO_LARGE, "hanh trinh 130° qua lon");
    Throttle_SetOpen(&thr, 280.0f);     /* 180°: không xác định chiều */
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_SPAN_TOO_LARGE, "hanh trinh 180° bi tu choi");
    CHECK(Throttle_SetClosed(&thr, NAN) == THR_CAL_BAD_INPUT, "NaN bi tu choi");
    CHECK(Throttle_LoadCalibration(&thr, 100.0f, 100.0f) == THR_CAL_SPAN_TOO_SMALL, "dong = mo bi tu choi");
    CHECK(thr.calibrated == 0U, "hieu chinh hong -> chua calibrated");

    /* 10) Đổi điểm hiệu chỉnh thì mất hiệu lực cho đến khi Commit */
    calib(100.0f, 150.0f);
    Throttle_Update(&thr, 125.0f, 1U);
    CHECK(thr.valid, "dang hop le");
    Throttle_SetClosed(&thr, 90.0f);
    Throttle_Update(&thr, 125.0f, 1U);
    CHECK(!thr.valid && thr.state == THR_NOT_CALIBRATED, "doi diem -> phai commit lai");
    CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_OK, "Commit lai OK (span 60)");

    /* 11) Cấu hình tùy chỉnh và cấu hình sai */
    {
        Throttle_Config_t c; Throttle_ConfigDefault(&c);
        c.dead_low_percent = 5.0f; c.dead_high_percent = 3.0f;
        CHECK(Throttle_Init(&thr, &c) == 0U, "cau hinh hop le");
        Throttle_LoadCalibration(&thr, 100.0f, 200.0f);          /* span 100 */
        Throttle_Update(&thr, 150.0f, 1U);                        /* raw 50 -> (50-5)/92*100 */
        CHECK(NEAR(thr.position_percent, 48.913f, 0.01f), "vung chet 5%/3% tinh dung");
        c.dead_low_percent = 30.0f; c.dead_high_percent = 30.0f;  /* tổng 60% */
        CHECK(Throttle_Init(&thr, &c) != 0U, "cau hinh sai bi tu choi");
        CHECK(NEAR(thr.cfg.dead_low_percent, 2.0f, 1e-6f), "cau hinh sai -> giu mac dinh");
        c.dead_low_percent = 2.0f; c.dead_high_percent = 2.0f; c.max_span_deg = 200.0f;
        CHECK(Throttle_Init(&thr, &c) != 0U, "max_span >= 180 bi tu choi");
    }

    /* 12) Trung bình vòng */
    {
        Throttle_Avg_t a; float m;
        Throttle_AvgReset(&a);
        CHECK(Throttle_AvgGet(&a, &m) == 0U, "chua co mau");
        Throttle_AvgAdd(&a, 359.0f); Throttle_AvgAdd(&a, 1.0f);
        CHECK(Throttle_AvgGet(&a, &m) == 1U && (m < 0.01f || m > 359.99f), "TB cua 359° và 1° ~ 0° (khong phai 180°)");
        Throttle_AvgReset(&a);
        Throttle_AvgAdd(&a, 100.0f); Throttle_AvgAdd(&a, 102.0f); Throttle_AvgAdd(&a, 98.0f);
        CHECK(Throttle_AvgGet(&a, &m) == 1U && NEAR(m, 100.0f, 0.01f), "TB thuong = 100°");
        Throttle_AvgAdd(&a, NAN);
        CHECK(a.count == 3U, "bo qua NaN");
    }


    /* 13) Lấy mẫu hiệu chỉnh tự động (Throttle_CalBegin / Throttle_CalStep) */
    {
        Throttle_Cal_t cal; Throttle_CalState_t s; uint32_t ms; int n;

        /* 13a) Tay ga đứng yên quanh 100°, nhiễu ±0.5°, gọi mỗi 1 ms, lấy mẫu mỗi 5 ms */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 50U, 5U, 2000U, 1000U);
        s = THR_CALSTEP_SAMPLING; n = 0;
        for (ms = 1000U; ms < 1000U + 2000U && s == THR_CALSTEP_SAMPLING; ms++, n++)
        {
            float noise = (((n * 37) % 11) - 5) * 0.1f;           /* -0.5..+0.5 */
            s = Throttle_CalStep(&cal, &thr, ms, 100.0f + noise, 1U);
        }
        CHECK(s == THR_CALSTEP_DONE && cal.result == THR_CAL_OK, "13a: lay mau xong");
        CHECK(NEAR(thr.closed_deg, 100.0f, 0.1f) && thr.closed_set == 1U, "13a: diem dong ~100°");
        CHECK(cal.avg.count == 50U, "13a: dung 50 mau");
        CHECK(n >= 245 && n <= 255, "13a: mat ~250 ms (50 mau x 5 ms)");

        /* 13b) Điểm mở hết quanh mốc 0°/360° (359.6..0.4) */
        Throttle_CalBegin(&cal, THR_POINT_OPEN, 20U, 5U, 2000U, 5000U);
        s = THR_CALSTEP_SAMPLING; n = 0;
        for (ms = 5000U; ms < 7000U && s == THR_CALSTEP_SAMPLING; ms++, n++)
        {
            float a = (n % 2) ? 359.6f : 0.4f;
            s = Throttle_CalStep(&cal, &thr, ms, a, 1U);
        }
        CHECK(s == THR_CALSTEP_DONE, "13b: xong");
        CHECK(thr.open_deg < 0.01f || thr.open_deg > 359.99f, "13b: diem mo ~0° (khong phai 180°)");

        /* 13c) Cả hai điểm xong -> Commit -> dùng được (đóng 100°, mở 150°) */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 10U, 5U, 1000U, 0U);
        for (ms = 0U; ms < 1000U && cal.state == THR_CALSTEP_SAMPLING; ms++) Throttle_CalStep(&cal, &thr, ms, 100.0f, 1U);
        Throttle_CalBegin(&cal, THR_POINT_OPEN, 10U, 5U, 1000U, 2000U);
        for (ms = 2000U; ms < 3000U && cal.state == THR_CALSTEP_SAMPLING; ms++) Throttle_CalStep(&cal, &thr, ms, 150.0f, 1U);
        CHECK(Throttle_CommitCalibration(&thr) == THR_CAL_OK, "13c: Commit OK");
        Throttle_Update(&thr, 125.0f, 1U);
        CHECK(thr.valid && NEAR(thr.raw_percent, 50.0f, 0.1f), "13c: 125° = 50%");

        /* 13d) Tay ga đang di chuyển (quét 100° -> 115°): không được chấp nhận */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 50U, 5U, 3000U, 0U);
        s = THR_CALSTEP_SAMPLING; n = 0;
        for (ms = 0U; ms < 3000U && s == THR_CALSTEP_SAMPLING; ms++, n++)
            s = Throttle_CalStep(&cal, &thr, ms, 100.0f + 15.0f * ((float)(n / 5) / 50.0f), 1U);
        CHECK(s == THR_CALSTEP_FAILED && cal.result == THR_CAL_UNSTABLE, "13d: tay ga di chuyen -> UNSTABLE");
        CHECK(thr.closed_set == 0U, "13d: khong ghi diem khi that bai");

        /* 13e) Góc luôn không hợp lệ (chưa có cảm biến) -> TIMEOUT, không đủ mẫu */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 50U, 5U, 500U, 0U);
        s = THR_CALSTEP_SAMPLING;
        for (ms = 0U; ms < 2000U && s == THR_CALSTEP_SAMPLING; ms++) s = Throttle_CalStep(&cal, &thr, ms, 0.0f, 0U);
        CHECK(s == THR_CALSTEP_FAILED && cal.result == THR_CAL_TIMEOUT && cal.avg.count == 0U, "13e: khong co mau hop le -> TIMEOUT");

        /* 13f) Mẫu không hợp lệ xen kẽ không được tính vào trung bình */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 20U, 5U, 3000U, 0U);
        s = THR_CALSTEP_SAMPLING; n = 0;
        for (ms = 0U; ms < 3000U && s == THR_CALSTEP_SAMPLING; ms++, n++)
        {   /* mỗi lần lấy mẫu: lần chẵn hợp lệ 80°, lần lẻ không hợp lệ (giá trị rác 300°) */
            uint8_t ok = (uint8_t)(((ms / 5U) % 2U) == 0U);
            s = Throttle_CalStep(&cal, &thr, ms, ok ? 80.0f : 300.0f, ok);
        }
        CHECK(s == THR_CALSTEP_DONE && NEAR(thr.closed_deg, 80.0f, 0.01f), "13f: mau khong hop le bi bo qua");


        /* 13g) Sau DONE/FAILED, gọi tiếp Step không đổi gì; millis tràn số vẫn đúng */
        CHECK(Throttle_CalStep(&cal, &thr, 99999U, 10.0f, 1U) == THR_CALSTEP_DONE, "13g: giu nguyen trang thai DONE");
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 10U, 5U, 1000U, 0xFFFFFF00U);   /* gần tràn uint32 */
        s = THR_CALSTEP_SAMPLING;
        for (ms = 0xFFFFFF00U; s == THR_CALSTEP_SAMPLING && (uint32_t)(ms - 0xFFFFFF00U) < 600U; ms++)
            s = Throttle_CalStep(&cal, &thr, ms, 60.0f, 1U);
        CHECK(s == THR_CALSTEP_DONE && NEAR(thr.closed_deg, 60.0f, 0.01f), "13g: tick tran so uint32 van dung");

        /* 13h) Tham số bất khả thi: timeout không đủ thời gian lấy mẫu -> báo lỗi ngay */
        Throttle_Init(&thr, NULL);
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 100U, 5U, 300U, 0U);       /* cần ~495 ms nhưng chỉ cho 300 ms */
        CHECK(cal.state == THR_CALSTEP_FAILED && cal.result == THR_CAL_BAD_INPUT, "13h: timeout qua ngan bi tu choi ngay");
        CHECK(Throttle_CalStep(&cal, &thr, 0U, 100.0f, 1U) == THR_CALSTEP_FAILED && cal.avg.count == 0U, "13h: Step khong lay mau sau khi bi tu choi");
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 100U, 5U, 495U, 0U);        /* đúng bằng (100-1)*5: vẫn bất khả thi */
        CHECK(cal.state == THR_CALSTEP_FAILED, "13h: timeout = (n-1)*interval bi tu choi");
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 100U, 5U, 496U, 0U);        /* vừa đủ */
        CHECK(cal.state == THR_CALSTEP_SAMPLING, "13h: timeout vua du duoc chap nhan");
        s = THR_CALSTEP_SAMPLING;
        for (ms = 0U; ms < 1000U && s == THR_CALSTEP_SAMPLING; ms++) s = Throttle_CalStep(&cal, &thr, ms, 100.0f, 1U);
        CHECK(s == THR_CALSTEP_DONE, "13h: timeout vua du thi lay du 100 mau va xong");
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 10U, 5U, 0U, 0U);          /* timeout 0 vô nghĩa */
        CHECK(cal.state == THR_CALSTEP_FAILED && cal.result == THR_CAL_BAD_INPUT, "13h: timeout 0 bi tu choi");
        Throttle_CalBegin(&cal, THR_POINT_CLOSED, 0U, 5U, 100U, 0U);         /* 0 mẫu -> coi là 1 mẫu */
        CHECK(cal.samples_needed == 1U && cal.state == THR_CALSTEP_SAMPLING, "13h: samples = 0 -> 1 mau");
    }

    printf("%s (fails=%d)\n", fails ? "CO LOI" : "TAT CA DAT", fails);
    return fails ? 1 : 0;
}