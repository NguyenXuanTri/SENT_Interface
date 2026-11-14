#include "throttle_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL: %s (dòng %d)\n", msg, __LINE__); fails++; } } while (0)
#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static double urand(void) { return (rand() + 1.0) / ((double)RAND_MAX + 2.0); }
static double gauss(void) { return sqrt(-2.0 * log(urand())) * cos(2.0 * 3.14159265358979 * urand()); }

/* chạy tới thời điểm t_end (mỗi 1 ms gọi 1 lần), đầu vào là hằng số x */
static void run_const(ThrFilter_t *f, uint32_t t0, uint32_t t1, float x)
{
    for (uint32_t t = t0; t <= t1; t++) ThrFilter_Update(f, x, 1U, t);
}

int main(void)
{
    ThrFilter_t f;

    /* 1) Nạp mẫu đầu: đầu ra bằng đúng mẫu, không trượt từ 0 lên */
    ThrFilter_Init(&f, NULL);
    CHECK(f.valid == 0U && f.output == 0.0f, "chua co mau -> dau ra 0, khong hop le");
    CHECK(ThrFilter_Update(&f, 37.5f, 1U, 1000U) == 1U && NEAR(f.output, 37.5f, 1e-5f) && f.valid, "mau dau = dau ra ngay");
    CHECK(f.output_permille == 375U, "permille 375");

    /* 2) Hằng số giữ nguyên */
    run_const(&f, 1001U, 3000U, 37.5f);
    CHECK(NEAR(f.output, 37.5f, 1e-4f), "dau vao hang -> dau ra hang");

    /* 3) Đáp ứng bậc thang 0 -> 50 */
    {
        ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c);
        for (int med = 1; med >= 0; med--)
        {
            c.median_enable = (uint8_t)med;
            ThrFilter_Init(&f, &c);
            ThrFilter_Update(&f, 0.0f, 1U, 0U);
            int t63 = -1; float prev = 0.0f; int mono = 1, over = 0;
            for (uint32_t t = 1; t <= 200; t++) {
                ThrFilter_Update(&f, 50.0f, 1U, t);
                if (f.output < prev - 1e-6f) mono = 0;
                if (f.output > 50.0f + 1e-4f) over = 1;
                if (t63 < 0 && f.output >= 31.6f) t63 = (int)t;
                prev = f.output;
            }
            printf("bac thang (median=%d): dat 63%% sau %d ms, cuoi=%.4f\n", med, t63, f.output);
            CHECK(mono && !over, "don dieu, khong vuot qua");
            CHECK(NEAR(f.output, 50.0f, 0.01f), "cuoi bac thang = 50");
            if (med) CHECK(t63 >= 10 && t63 <= 14, "median bat: 63% trong 10..14 ms"); else CHECK(t63 >= 8 && t63 <= 12, "median tat: 63% trong 8..12 ms");
        }
    }

    /* 4) Gai đơn lẻ (đúng 1 mẫu của bộ lọc) bị median loại; 2 mẫu liên tiếp thì không */
    {
        for (int med = 1; med >= 0; med--)
        {
            ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c); c.median_enable = (uint8_t)med;
            ThrFilter_Init(&f, &c);
            /* 1 mẫu xấu (80) đúng tại t=102: mẫu của bộ lọc rơi vào các t chẵn */
            ThrFilter_Init(&f, &c); run_const(&f, 0U, 100U, 30.0f);
            float peak = 0.0f; for (uint32_t t = 101; t <= 200; t++) { float x = (t == 102U) ? 80.0f : 30.0f; ThrFilter_Update(&f, x, 1U, t); if (fabsf(f.output - 30.0f) > peak) peak = fabsf(f.output - 30.0f); }
            printf("gai 1 mau (median=%d): lech lon nhat %.3f\n", med, peak);
            if (med) CHECK(peak < 0.01f, "median loai han gai 1 mau"); else CHECK(peak > 5.0f, "khong median: gai lot qua (de ghep thu co y nghia)");
        }
        ThrFilter_Init(&f, NULL); run_const(&f, 0U, 100U, 30.0f);
        float peak = 0.0f; for (uint32_t t = 101; t <= 200; t++) { float x = (t >= 102U && t <= 105U) ? 80.0f : 30.0f; ThrFilter_Update(&f, x, 1U, t); if (f.output - 30.0f > peak) peak = f.output - 30.0f; }
        printf("gai 2 mau lien tiep (median=1): lech lon nhat %.3f\n", peak);
        CHECK(peak > 1.0f, "2 mau xau lien tiep thi median khong loai duoc (gioi han da biet)");
    }

    /* 5) Giảm nhiễu: 40 % + nhiễu Gauss sigma = 1 % */
    {
        srand(7);
        for (int med = 1; med >= 0; med--)
        {
            ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c); c.median_enable = (uint8_t)med;
            ThrFilter_Init(&f, &c);
            double s_in = 0, s_out = 0, m_out = 0; int n = 0;
            for (uint32_t t = 0; t < 8000; t++) {
                float x = 40.0f + (float)gauss();
                ThrFilter_Update(&f, x, 1U, t);
                if (t >= 500) { s_in += (x - 40.0) * (x - 40.0); m_out += f.output; n++; }
            }
            m_out /= n; srand(7);
            ThrFilter_Init(&f, &c);
            for (uint32_t t = 0; t < 8000; t++) {
                float x = 40.0f + (float)gauss();
                ThrFilter_Update(&f, x, 1U, t);
                if (t >= 500) s_out += (f.output - m_out) * (f.output - m_out);
            }
            double r = sqrt(s_out / n) / sqrt(s_in / n);
            printf("giam nhieu (median=%d): sigma ra / sigma vao = %.3f\n", med, r);
            CHECK(r < (med ? 0.40 : 0.50), "nhieu giam du manh");
        }
    }

    /* 6) Không hợp lệ -> đặt lại; hợp lệ trở lại thì nạp mẫu mới ngay, không trượt từ giá trị cũ */
    ThrFilter_Init(&f, NULL);
    run_const(&f, 0U, 500U, 60.0f);
    CHECK(NEAR(f.output, 60.0f, 1e-3f) && f.valid, "dang hop le o 60%");
    CHECK(ThrFilter_Update(&f, 60.0f, 0U, 501U) == 0U && f.valid == 0U && f.output == 0.0f && f.output_permille == 0U, "khong hop le -> 0, valid=0");
    CHECK(ThrFilter_Update(&f, 20.0f, 1U, 502U) == 1U && NEAR(f.output, 20.0f, 1e-5f), "hop le lai -> nap mau moi, khong truoc tu 60");
    ThrFilter_Update(&f, NAN, 1U, 504U);
    CHECK(f.valid == 0U && f.output == 0.0f, "NaN -> khong hop le");
    ThrFilter_Update(&f, INFINITY, 1U, 506U);
    CHECK(f.valid == 0U, "vo han -> khong hop le");

    /* 7) Lâu không cập nhật (dữ liệu cũ) -> nạp lại */
    ThrFilter_Init(&f, NULL);
    ThrFilter_Update(&f, 10.0f, 1U, 0U);
    ThrFilter_Update(&f, 90.0f, 1U, 200U);
    CHECK(NEAR(f.output, 90.0f, 1e-4f), "khoang trong > max_gap -> nap lai mau moi");

    /* 8) Chu kỳ lấy mẫu: gọi sớm hơn period_ms thì không đổi đầu ra */
    {
        ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c); c.median_enable = 0U;
        ThrFilter_Init(&f, &c);
        ThrFilter_Update(&f, 10.0f, 1U, 1000U);
        ThrFilter_Update(&f, 50.0f, 1U, 1001U);
        CHECK(NEAR(f.output, 10.0f, 1e-6f), "cach 1 ms < period 2 ms: giu nguyen");
        ThrFilter_Update(&f, 50.0f, 1U, 1002U);
        CHECK(f.output > 10.0f + 1.0f, "du 2 ms: cap nhat");
        float a = f.output; ThrFilter_Update(&f, 50.0f, 1U, 1002U); ThrFilter_Update(&f, 50.0f, 1U, 1002U);
        CHECK(f.output == a, "goi lap cung thoi diem khong lam doi ket qua");
    }

    /* 9) Bộ đếm ms tràn số uint32 cho kết quả y hệt */
    {
        ThrFilter_t a, b; ThrFilter_Init(&a, NULL); ThrFilter_Init(&b, NULL);
        srand(3); int same = 1; uint32_t base = 0xFFFFFFF0U;
        for (uint32_t t = 0; t < 2000; t++) {
            float x = (float)(50.0 + 30.0 * sin(t * 0.01) + gauss());
            ThrFilter_Update(&a, x, 1U, t); ThrFilter_Update(&b, x, 1U, base + t);
            if (a.output != b.output || a.valid != b.valid) same = 0;
        }
        CHECK(same, "tran so ms: ket qua giong het khi khong tran");
    }

    /* 10) Kiểm tra cấu hình */
    {
        ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c);
        CHECK(ThrFilter_Init(&f, &c) == 0U, "mac dinh hop le");
        c.cutoff_hz = 0.0f;   CHECK(ThrFilter_Init(&f, &c) != 0U, "cutoff 0 bi tu choi");
        CHECK(NEAR(f.cfg.cutoff_hz, 20.0f, 1e-6f), "cau hinh sai -> giu mac dinh");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = -5.0f;  CHECK(ThrFilter_Init(&f, &c) != 0U, "cutoff am");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = NAN;    CHECK(ThrFilter_Init(&f, &c) != 0U, "cutoff NaN");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = 200.0f; CHECK(ThrFilter_Init(&f, &c) != 0U, "cutoff > fs/4 (125 Hz)");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = 125.0f; CHECK(ThrFilter_Init(&f, &c) == 0U, "cutoff = fs/4 duoc chap nhan");
        ThrFilter_ConfigDefault(&c); c.period_ms = 0U;     CHECK(ThrFilter_Init(&f, &c) != 0U, "period 0");
        ThrFilter_ConfigDefault(&c); c.max_gap_ms = 3U;    CHECK(ThrFilter_Init(&f, &c) != 0U, "max_gap < 2*period");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = 5.0f; c.period_ms = 5U; c.max_gap_ms = 100U;
        CHECK(ThrFilter_Init(&f, &c) == 0U && NEAR(f.cfg.cutoff_hz, 5.0f, 1e-6f), "cau hinh tuy chinh hop le");
        ThrFilter_ConfigDefault(&c); c.cutoff_hz = 60.0f; c.period_ms = 5U; c.max_gap_ms = 100U;
        CHECK(ThrFilter_Init(&f, &c) != 0U, "cutoff 60 Hz với period 5 ms (fs/4 = 50)");
    }

    /* 11) Đáp ứng tần số: sin 50 +- 20 % */
    {
        double fr[2] = { 5.0, 100.0 };
        for (int k = 0; k < 2; k++) {
            ThrFilter_Init(&f, NULL);
            double lo = 1e9, hi = -1e9;
            for (uint32_t t = 0; t < 4000; t++) {
                float x = (float)(50.0 + 20.0 * sin(2.0 * 3.14159265358979 * fr[k] * t / 1000.0));
                ThrFilter_Update(&f, x, 1U, t);
                if (t >= 2000) { if (f.output < lo) lo = f.output; if (f.output > hi) hi = f.output; }
            }
            double gain = (hi - lo) / 40.0;
            printf("dap ung tan so %.0f Hz: do loi = %.3f\n", fr[k], gain);
            if (k == 0) CHECK(gain > 0.90 && gain < 1.01, "5 Hz di qua gan nhu nguyen ven");
            else        CHECK(gain < 0.30, "100 Hz bi suy giam manh");
        }
    }

    /* 12) So sánh với bản tham chiếu độc lập (double) trên chuỗi ngẫu nhiên */
    {
        srand(99);
        long steps = 0, mism = 0, vmism = 0; double maxerr = 0; int range_bad = 0;
        for (int seq = 0; seq < 20000; seq++) {
            ThrFilter_Config_t c; ThrFilter_ConfigDefault(&c);
            c.period_ms = 1U + (uint32_t)(rand() % 6);
            c.max_gap_ms = 2U * c.period_ms + (uint32_t)(rand() % 60);
            c.cutoff_hz = (float)(0.5 + urand() * ((1000.0 / c.period_ms) / 4.0 - 0.5));
            c.median_enable = (uint8_t)(rand() & 1);
            if (ThrFilter_Init(&f, &c) != 0U) { printf("cfg hop le bi tu choi\n"); return 2; }
            /* tham chiếu */
            int seeded = 0; double y = 0, w[3] = {0,0,0}; uint32_t last = 0; int rvalid = 0; double rout = 0;
            double tau = 1.0 / (2.0 * 3.14159265358979 * (double)f.cfg.cutoff_hz);
            uint32_t t = (uint32_t)rand();
            for (int i = 0; i < 400; i++) {
                int r = rand() % 100;
                t += (r < 3) ? (uint32_t)(c.max_gap_ms + 1 + rand() % 50) : (uint32_t)(rand() % 6);
                int valid = (rand() % 100) >= 3;
                double x = urand() * 130.0 - 15.0;               /* có cả ngoài 0..100 */
                ThrFilter_Update(&f, (float)x, (uint8_t)valid, t);
                double xf = (double)(float)x;
                if (!valid) { seeded = 0; rvalid = 0; rout = 0; }
                else {
                    xf = xf < 0 ? 0 : (xf > 100 ? 100 : xf);
                    if (!seeded) { y = w[0] = w[1] = w[2] = xf; last = t; seeded = 1; rvalid = 1; rout = y; }
                    else {
                        uint32_t dt = t - last;
                        if (dt < c.period_ms) { }
                        else if (dt > c.max_gap_ms) { y = w[0] = w[1] = w[2] = xf; last = t; rout = y; }
                        else {
                            last = t; w[0] = w[1]; w[1] = w[2]; w[2] = xf;
                            double m = xf;
                            if (c.median_enable) { double a = w[0], b = w[1], cc = w[2]; double lo = a < b ? a : b, hi = a < b ? b : a; double mm = cc < hi ? cc : hi; m = lo > mm ? lo : mm; }
                            double al = (dt * 0.001) / (tau + dt * 0.001); y += al * (m - y); rout = y;
                        }
                    }
                }
                steps++;
                if (f.valid != (uint8_t)rvalid) vmism++;
                double e = fabs(f.output - rout); if (e > maxerr) maxerr = e;
                if (e > 2e-3) mism++;
                if (!(f.output >= 0.0f && f.output <= 100.0f) || f.output_permille > 1000U) range_bad++;
            }
        }
        printf("so sanh tham chieu: %ld buoc, lech valid=%ld, lech gia tri > 0.002=%ld, sai lech lon nhat = %.6f, ngoai dai = %d\n", steps, vmism, mism, maxerr, range_bad);
        CHECK(vmism == 0 && mism == 0 && range_bad == 0, "khop ban tham chieu doc lap tren moi buoc");
    }

    printf("%s (fails=%d)\n", fails ? "CO LOI" : "TAT CA DAT", fails);
    return fails ? 1 : 0;
}