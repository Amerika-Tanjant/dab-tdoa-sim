% =========================================================================
% ADIM 2: DAB SİNYAL MODELİ
% DAB SoOP TDOA Localization
% =========================================================================
% Bu dosya DAB çerçeve yapısını oluşturur:
%   1. Null Sembolü  → kaba zamanlama için
%   2. PRS Sembolü   → hassas zamanlama/korelasyon için
%   3. Veri Sembolleri → gerçekçilik için (basitleştirilmiş)
%
% Çıktı: 'dab_signal_data.mat' dosyasına kaydedilir
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 2: DAB Sinyal Modeli Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: DAB STANDART PARAMETRELERİ (ETSI EN 300 401 - Mode 1)
% =========================================================================
% DAB'ın 4 modu var (Mode 1-4). Biz Mode 1 kullanıyoruz.
% Mode 1: En geniş kapsama alanı, SoOP için en uygun.

Fs = 2.048e6;       % Örnekleme frekansı: 2.048 MHz [örnekleme/saniye]

% --- Sembol Uzunlukları (örnek sayısı cinsinden) ---
% DAB Mode 1 standart değerleri:
T_null_ms   = 1.297;    % Null sembolünün süresi [milisaniye]
T_prs_ms    = 1.246;    % PRS sembolünün süresi [milisaniye]
T_data_ms   = 1.246;    % Veri sembolü süresi [milisaniye]
N_data_syms = 76;       % Bir çerçevedeki veri sembolü sayısı

% Milisaniyeden örnek sayısına çevir: N = T[ms] * Fs / 1000
N_null = round(T_null_ms * Fs / 1000);   % ~2656 örnek
N_prs  = round(T_prs_ms  * Fs / 1000);  % ~2552 örnek
N_data = round(T_data_ms * Fs / 1000);  % ~2552 örnek

% Bir tam çerçevenin toplam örnek sayısı
N_frame = N_null + N_prs + N_data_syms * N_data;

fprintf('DAB Mode 1 Parametreleri:\n');
fprintf('  Örnekleme frekansı : %.3f MHz\n', Fs/1e6);
fprintf('  Null sembol süresi : %.3f ms (%d örnek)\n', T_null_ms, N_null);
fprintf('  PRS sembol süresi  : %.3f ms (%d örnek)\n', T_prs_ms,  N_prs);
fprintf('  Veri sembol sayısı : %d\n', N_data_syms);
fprintf('  Toplam çerçeve     : %d örnek (%.1f ms)\n\n', ...
        N_frame, N_frame/Fs*1000);

% =========================================================================
% BÖLÜM 2: NULL SEMBOLÜ OLUŞTURMA
% =========================================================================
% Null sembolü = tamamen sıfır (sessizlik).
% Alıcı bu enerji düşüşünü tespit ederek çerçeve başını bulur.
% Gerçek DAB'da tam sıfır değil ama biz idealleştiriyoruz.

null_symbol = zeros(1, N_null);   % N_null adet sıfır

fprintf('Null Sembolü:\n');
fprintf('  Uzunluk  : %d örnek\n', length(null_symbol));
fprintf('  Enerji   : %.4f (sıfır olmalı)\n', sum(abs(null_symbol).^2));
fprintf('  Amaç     : Çerçeve başını işaretler (enerji düşüşü)\n\n');

% =========================================================================
% BÖLÜM 3: PRS (PHASE REFERENCE SYMBOL) OLUŞTURMA
% =========================================================================
% PRS = Faz Referans Sembolü
% Bu sinyal korelasyon için kullanılır. Alıcı bu "imzayı" arar.
%
% Gerçek DAB PRS'i çok karmaşık (OFDM tabanlı, faz kodlamalı).
% Biz simülasyon için deterministik pseudo-random bir dizi kullanıyoruz.
% Önemli olan: Her TX için AYNI ve BİLİNEN bir dizi olması.
%
% Nasıl çalışır?
%   Alıcı → aldığı sinyal ile PRS'i çapraz-korelasyona sokar
%   Korelasyon tepe noktası → sinyalin kaç örnek gecikmeli geldiğini söyler
%   Bu gecikme → TOA (Varış Zamanı) kestirimi

rng(42);    % Rastgele sayı üretecini sabitle (42 = seed)
            % Seed sabitlenince her çalıştırmada AYNI PRS oluşur
            % Bu kritik! Alıcı ve verici aynı PRS'i bilmeli.

% BPSK modülasyonu: +1 ve -1 değerlerinden oluşan dizi
% randn > 0 ise +1, değilse -1 → Basit BPSK
prs_bits = sign(randn(1, N_prs));   % N_prs adet +1 veya -1

% Normalize et: Enerji = 1 olsun
prs_symbol = prs_bits / sqrt(mean(prs_bits.^2));

fprintf('PRS (Phase Reference Symbol):\n');
fprintf('  Uzunluk     : %d örnek\n', length(prs_symbol));
fprintf('  Enerji      : %.4f (1 olmalı)\n', mean(prs_symbol.^2));
fprintf('  Değer aralığı: [%.1f, %.1f]\n', min(prs_symbol), max(prs_symbol));
fprintf('  Amaç        : Korelasyon ile TOA kestirimi\n\n');

% =========================================================================
% BÖLÜM 4: VERİ SEMBOLLERİ OLUŞTURMA
% =========================================================================
% Gerçek DAB'da veri sembolleri ses/müzik bilgisi taşır.
% Simülasyonda bunu basitleştiriyoruz: rastgele BPSK veri.
% DAE için bu kısım çok önemli değil, TOA kestirimi PRS'den yapılıyor.

data_symbols = sign(randn(1, N_data * N_data_syms));  % Rastgele veri
data_symbols = data_symbols / sqrt(mean(data_symbols.^2));  % Normalize

fprintf('Veri Sembolleri:\n');
fprintf('  Toplam uzunluk: %d örnek (%d sembol x %d örnek)\n', ...
        length(data_symbols), N_data_syms, N_data);
fprintf('  Amaç          : Gerçekçilik (TOA için kullanılmaz)\n\n');

% =========================================================================
% BÖLÜM 5: TAM DAB ÇERÇEVESİNİ BİRLEŞTİR
% =========================================================================
% Çerçeve yapısı (soldan sağa):
%   [NULL] [PRS] [DATA_1] [DATA_2] ... [DATA_76]
%    ↑      ↑     ↑
%   Sıfır  İmza  Ses verisi

dab_frame = [null_symbol, prs_symbol, data_symbols];

fprintf('Tam DAB Çerçevesi:\n');
fprintf('  Toplam uzunluk: %d örnek\n', length(dab_frame));
fprintf('  Süre          : %.2f ms\n', length(dab_frame)/Fs*1000);
fprintf('  Yapı          : [NULL(%d)] + [PRS(%d)] + [DATA(%d)]\n\n', ...
        N_null, N_prs, length(data_symbols));

% =========================================================================
% BÖLÜM 6: PRS KORELASYONUNU TEST ET
% =========================================================================
% Simülasyonun doğruluğunu kontrol edelim.
% Temiz sinyalde (gürültüsüz) PRS korelasyonu tam olarak doğru yerde tepe vermeli.
%
% Korelasyon nasıl çalışır?
%   - Alınan sinyal ile referans PRS'i kaydırarak çarpıyoruz
%   - Sinyaller örtüştüğünde maksimum değer oluşur
%   - Bu maksimumun konumu = gecikme = TOA

% PRS'in çerçeve içindeki başlangıç indeksi
prs_start_idx = N_null + 1;   % Null'dan sonra PRS başlar

% Test: Çerçeveden PRS bölgesini çıkar ve korelasyon yap
test_segment = dab_frame(prs_start_idx : prs_start_idx + N_prs - 1);

% xcorr: Çapraz korelasyon fonksiyonu
% 'normalized': Tepe değerini 1'e normalize eder
[corr_vals, corr_lags] = xcorr(test_segment, prs_symbol, 'normalized');

% Maksimum korelasyon noktası
[max_corr, max_idx] = max(abs(corr_vals));
peak_lag = corr_lags(max_idx);   % Gecikme (örnek cinsinden)

fprintf('Korelasyon Testi (Gürültüsüz):\n');
fprintf('  Maksimum korelasyon: %.4f (1.0 olmalı)\n', max_corr);
fprintf('  Tepe noktası lag   : %d örnek (0 olmalı)\n', peak_lag);
if abs(peak_lag) == 0 && max_corr > 0.99
    fprintf('  Sonuç: ✓ BAŞARILI - PRS doğru tespit edildi\n\n');
else
    fprintf('  Sonuç: ✗ HATA - Kontrol et\n\n');
end

% =========================================================================
% BÖLÜM 7: VERİLERİ KAYDET
% =========================================================================

save('dab_signal_data.mat', ...
    'dab_frame', ...        % Tam DAB çerçevesi
    'null_symbol', ...      % Null sembolü
    'prs_symbol', ...       % PRS sembolü (korelasyon referansı)
    'data_symbols', ...     % Veri sembolleri
    'N_null', ...           % Null sembol uzunluğu
    'N_prs', ...            % PRS uzunluğu
    'N_data', ...           % Veri sembol uzunluğu
    'N_data_syms', ...      % Veri sembol sayısı
    'N_frame', ...          % Toplam çerçeve uzunluğu
    'Fs', ...               % Örnekleme frekansı
    'prs_start_idx' ...     % PRS başlangıç indeksi
);

fprintf('✓ Veriler "dab_signal_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 8: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 2: DAB Sinyal Modeli', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1200, 800]);

% --- Panel 1: Tam Çerçeve ---
subplot(3, 2, [1,2]);
t_frame = (0:length(dab_frame)-1) / Fs * 1000;   % Zaman ekseni [ms]
plot(t_frame, dab_frame, 'b-', 'LineWidth', 0.5);
hold on;

% Bölge işaretleri
null_end_t   = N_null / Fs * 1000;
prs_end_t    = (N_null + N_prs) / Fs * 1000;

% Renkli arka plan: Null=kırmızı, PRS=yeşil, Data=mavi
patch([0, null_end_t, null_end_t, 0], [-1.5, -1.5, 1.5, 1.5], ...
      'red', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
patch([null_end_t, prs_end_t, prs_end_t, null_end_t], [-1.5, -1.5, 1.5, 1.5], ...
      'green', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
patch([prs_end_t, t_frame(end), t_frame(end), prs_end_t], [-1.5, -1.5, 1.5, 1.5], ...
      'blue', 'FaceAlpha', 0.08, 'EdgeColor', 'none');

% Etiketler
text(null_end_t/2, 1.2, 'NULL', 'HorizontalAlignment', 'center', ...
     'FontWeight', 'bold', 'Color', 'red', 'FontSize', 11);
text((null_end_t+prs_end_t)/2, 1.2, 'PRS', 'HorizontalAlignment', 'center', ...
     'FontWeight', 'bold', 'Color', [0 0.5 0], 'FontSize', 11);
text((prs_end_t+t_frame(end))/2, 1.2, 'VERİ SEMBOLLERİ (x76)', ...
     'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
     'Color', 'blue', 'FontSize', 11);

xlabel('Zaman [ms]', 'FontSize', 11);
ylabel('Genlik', 'FontSize', 11);
title('DAB Çerçeve Yapısı (Tam Çerçeve)', 'FontSize', 13, 'FontWeight', 'bold');
ylim([-1.5, 1.5]);
grid on;

% --- Panel 2: Null Sembolü Yakın Plan ---
subplot(3, 2, 3);
t_null = (0:N_null-1) / Fs * 1000;
plot(t_null, null_symbol, 'r-', 'LineWidth', 1.5);
xlabel('Zaman [ms]', 'FontSize', 10);
ylabel('Genlik', 'FontSize', 10);
title(sprintf('NULL Sembolü (%d örnek)', N_null), 'FontSize', 11, 'FontWeight', 'bold');
ylim([-0.1, 0.1]);
grid on;

% --- Panel 3: PRS Yakın Plan ---
subplot(3, 2, 4);
t_prs = (0:N_prs-1) / Fs * 1000;
plot(t_prs, prs_symbol, 'g-', 'LineWidth', 1);
xlabel('Zaman [ms]', 'FontSize', 10);
ylabel('Genlik', 'FontSize', 10);
title(sprintf('PRS Sembolü (%d örnek)', N_prs), 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% --- Panel 4: Korelasyon Sonucu ---
subplot(3, 2, 5);
plot(corr_lags, abs(corr_vals), 'b-', 'LineWidth', 1.5);
hold on;
plot(peak_lag, max_corr, 'r*', 'MarkerSize', 15, 'DisplayName', 'Tepe Noktası');
xlabel('Lag (Gecikme) [örnek]', 'FontSize', 10);
ylabel('Korelasyon', 'FontSize', 10);
title('PRS Çapraz Korelasyonu (Gürültüsüz)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Korelasyon', 'Tepe', 'Location', 'best');
grid on;
xlim([-100, 100]);

% --- Panel 5: PRS Frekans Spektrumu ---
subplot(3, 2, 6);
PRS_FFT = abs(fft(prs_symbol));
f_axis = (0:N_prs-1) * Fs / N_prs / 1e3;   % kHz cinsinden
plot(f_axis(1:N_prs/2), PRS_FFT(1:N_prs/2), 'm-', 'LineWidth', 1);
xlabel('Frekans [kHz]', 'FontSize', 10);
ylabel('Genlik', 'FontSize', 10);
title('PRS Frekans Spektrumu', 'FontSize', 11, 'FontWeight', 'bold');
grid on;

sgtitle('ADIM 2: DAB Sinyal Modeli', 'FontSize', 15, 'FontWeight', 'bold');

saveas(gcf, 'step2_dab_signal.png');
fprintf('✓ Grafik "step2_dab_signal.png" olarak kaydedildi.\n');

% =========================================================================
% ÖZET
% =========================================================================
fprintf('\n========================================\n');
fprintf('  ADIM 2 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Oluşturulan değişkenler:\n');
fprintf('  dab_frame  : 1x%d (tam çerçeve)\n', length(dab_frame));
fprintf('  null_symbol: 1x%d (null)\n', length(null_symbol));
fprintf('  prs_symbol : 1x%d (PRS - korelasyon imzası)\n', length(prs_symbol));
fprintf('\nSonraki adım: step3_channel.m\n');
fprintf('========================================\n');