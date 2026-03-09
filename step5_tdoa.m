% =========================================================================
% ADIM 5: DİFERANSİYEL TDOA HESAPLAMA
% DAB SoOP TDOA Localization
% =========================================================================
% TDOA = Time Difference of Arrival (Varış Zamanı Farkı)
%
% Diferansiyel TDOA formülü:
%   Δτ_i = (TOA_UAV_i - TOA_RefRX_i)  →  vericinin saat hatasını iptal eder
%
% Bu çıkarma işlemi şunları iptal eder:
%   - Verici tarafındaki saat kayması
%   - Verici tarafındaki yayın gecikmesi
%   - Verici tarafındaki jitter
%
% Sonuç: UAV'ın konumunu hesaplamak için kullanılacak 4 TDOA değeri
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 5: TDOA Hesaplama Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('dab_signal_data.mat');
load('toa_data.mat');

fprintf('✓ Tüm veriler yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: DİFERANSİYEL TDOA HESAPLA
% =========================================================================
% Formül: Δτ_i = TOA_UAV_i - TOA_RefRX_i
%
% Fiziksel anlamı:
%   TOA_UAV_i   : TX_i'den UAV'a olan mesafeye karşılık gelen zaman
%   TOA_RefRX_i : TX_i'den Ref_RX'e olan mesafeye karşılık gelen zaman
%   Fark        : UAV ile Ref_RX arasındaki mesafe farkına karşılık gelir
%
% Örnekten metreye çeviri: mesafe = TDOA_örnek * c / Fs

% --- Ölçülen (gürültülü) TDOA ---
% TOA_UAV_phat : [N_pos x 4] — her konum için 4 TX'in TOA'sı
% TOA_RefRX_phat: [1 x 4]   — Ref RX'in 4 TX'ten TOA'sı
TDOA_measured = zeros(N_pos, 4);   % [N_pos x 4] örnek cinsinden

for pos = 1:N_pos
    for tx = 1:4
        % Diferansiyel çıkarma: UAV TOA - Ref RX TOA
        TDOA_measured(pos, tx) = TOA_UAV_phat(pos, tx) - TOA_RefRX_phat(tx);
    end
end

% Örnek → metre
TDOA_measured_m = TDOA_measured * c / Fs;

% --- Gerçek (gürültüsüz) TDOA ---
% Geometriden hesaplanan ideal değerler
TDOA_true = zeros(N_pos, 4);

for pos = 1:N_pos
    for tx = 1:4
        % Gerçek mesafe farkı: (UAV-TX mesafesi) - (RefRX-TX mesafesi)
        TDOA_true(pos, tx) = (d_TX_UAV(pos,tx) - d_TX_RefRX(tx)) / c * Fs;
    end
end

TDOA_true_m = TDOA_true * c / Fs;   % Örnek → metre

% --- TDOA hatası ---
TDOA_error    = TDOA_measured - TDOA_true;          % örnek
TDOA_error_m  = TDOA_error * c / Fs;                % metre
TDOA_error_ns = TDOA_error / Fs * 1e9;              % nanosaniye

fprintf('TDOA Değerleri (Konum 1, tüm TX):\n');
fprintf('  %-5s  %-15s  %-15s  %-15s\n', 'TX', 'Gerçek [m]', 'Ölçülen [m]', 'Hata [m]');
fprintf('  %-5s  %-15s  %-15s  %-15s\n', '---', '---------------', '---------------', '---------------');
for tx = 1:4
    fprintf('  TX%-3d  %-15.2f  %-15.2f  %-15.2f\n', tx, ...
            TDOA_true_m(1,tx), TDOA_measured_m(1,tx), TDOA_error_m(1,tx));
end

fprintf('\nGenel TDOA Hata İstatistikleri:\n');
fprintf('  Ortalama hata : %.2f m\n', mean(abs(TDOA_error_m(:))));
fprintf('  Std sapma     : %.2f m\n', std(TDOA_error_m(:)));
fprintf('  Maks hata     : %.2f m\n\n', max(abs(TDOA_error_m(:))));

% =========================================================================
% BÖLÜM 3: TDOA GEOMETRİSİNİ GÖRSELLEŞTIR
% =========================================================================
% Her TDOA ölçümü bir hiperbol tanımlar.
% UAV bu hiperbolların kesişim noktasındadır.
% Burada sadece kavramı gösteriyoruz (çizimi basitleştirdik).

fprintf('TDOA Hiperbolleri hesaplanıyor...\n');

% Örnek: Konum 1 için TX1-TX2 çifti arasındaki hiperbol
% d(UAV, TX1) - d(UAV, TX2) = TDOA_12 * c
% Bu denklem bir hiperbol tanımlar

% Izgara oluştur (hiperbol çizmek için)
[X_grid, Y_grid] = meshgrid(linspace(-200, 2200, 300), ...
                             linspace(-700, 2700, 300));
Z_grid = 200 * ones(size(X_grid));   % UAV irtifası sabit

% TX1 ve TX2 için mesafe farkı hesapla (grid üzerinde)
d1 = sqrt((X_grid - TX(1,1)).^2 + (Y_grid - TX(1,2)).^2 + (Z_grid - TX(1,3)).^2);
d2 = sqrt((X_grid - TX(2,1)).^2 + (Y_grid - TX(2,2)).^2 + (Z_grid - TX(2,3)).^2);
d3 = sqrt((X_grid - TX(3,1)).^2 + (Y_grid - TX(3,2)).^2 + (Z_grid - TX(3,3)).^2);
d4 = sqrt((X_grid - TX(4,1)).^2 + (Y_grid - TX(4,2)).^2 + (Z_grid - TX(4,3)).^2);

% Ref RX mesafeleri (sabit)
dr1 = d_TX_RefRX(1);
dr2 = d_TX_RefRX(2);
dr3 = d_TX_RefRX(3);
dr4 = d_TX_RefRX(4);

% TDOA hiperbol denklemleri (gerçek değerlerle)
H12 = (d1 - dr1) - (d2 - dr2);   % TDOA_1 - TDOA_2 hiperbolü
H13 = (d1 - dr1) - (d3 - dr3);
H14 = (d1 - dr1) - (d4 - dr4);

fprintf('✓ Hiperbol hesaplandı.\n\n');

% =========================================================================
% BÖLÜM 4: KAYDET
% =========================================================================

save('tdoa_data.mat', ...
    'TDOA_measured', ...    % Ölçülen TDOA [örnek]      [N_pos x 4]
    'TDOA_measured_m', ...  % Ölçülen TDOA [metre]      [N_pos x 4]
    'TDOA_true', ...        % Gerçek TDOA [örnek]       [N_pos x 4]
    'TDOA_true_m', ...      % Gerçek TDOA [metre]       [N_pos x 4]
    'TDOA_error_m', ...     % TDOA hatası [metre]       [N_pos x 4]
    'TDOA_error_ns' ...     % TDOA hatası [nanosaniye]  [N_pos x 4]
);

fprintf('✓ Veriler "tdoa_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 5: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 5: TDOA Hesaplama', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1200, 800]);

% --- Panel 1: TDOA Hiperbolları (2D) ---
subplot(2, 3, [1, 2]);
hold on;

% Hiperbolları çiz (kontur = sıfır geçiş noktaları)
ref_tdoa_12 = TDOA_true_m(1,1) - TDOA_true_m(1,2);
ref_tdoa_13 = TDOA_true_m(1,1) - TDOA_true_m(1,3);
ref_tdoa_14 = TDOA_true_m(1,1) - TDOA_true_m(1,4);

contour(X_grid, Y_grid, H12, [ref_tdoa_12, ref_tdoa_12], ...
        'r-', 'LineWidth', 2);
contour(X_grid, Y_grid, H13, [ref_tdoa_13, ref_tdoa_13], ...
        'b-', 'LineWidth', 2);
contour(X_grid, Y_grid, H14, [ref_tdoa_14, ref_tdoa_14], ...
        'g-', 'LineWidth', 2);

% TX'leri çiz
scatter(TX(:,1), TX(:,2), 200, 'bs', 'filled');
for m = 1:4
    text(TX(m,1)+60, TX(m,2)+60, sprintf('TX%d', m), ...
         'FontSize', 10, 'Color', 'blue', 'FontWeight', 'bold');
end

% Ref RX
scatter(Ref_RX(1), Ref_RX(2), 200, 'g^', 'filled');
text(Ref_RX(1)+60, Ref_RX(2)+60, 'Ref RX', ...
     'FontSize', 10, 'Color', 'green', 'FontWeight', 'bold');

% UAV yörüngesi
plot(UAV_true(:,1), UAV_true(:,2), 'k--', 'LineWidth', 1.5);
scatter(UAV_true(:,1), UAV_true(:,2), 40, 'ko', 'filled');

% Konum 1'i vurgula
scatter(UAV_true(1,1), UAV_true(1,2), 150, 'r*', 'MarkerEdgeColor', 'r');
text(UAV_true(1,1)+60, UAV_true(1,2)+60, 'UAV[1]', ...
     'FontSize', 10, 'Color', 'red', 'FontWeight', 'bold');

legend({'H(TX1-TX2)', 'H(TX1-TX3)', 'H(TX1-TX4)', ...
        'TX', '', '', '', 'Ref RX', 'UAV Yörüngesi', '', 'UAV[1]'}, ...
       'Location', 'northeast', 'FontSize', 8);

xlabel('X [m]', 'FontSize', 11);
ylabel('Y [m]', 'FontSize', 11);
title('TDOA Hiperbolleri - Kesişim = UAV Konumu', ...
      'FontSize', 12, 'FontWeight', 'bold');
axis equal;
grid on;
xlim([-200, 2200]);
ylim([-700, 2700]);

% --- Panel 3: TDOA Hata Dağılımı ---
subplot(2, 3, 3);
histogram(TDOA_error_m(:), 20, 'FaceColor', [1 0.5 0], 'EdgeColor', 'white');
xline(mean(TDOA_error_m(:)), 'r--', 'LineWidth', 2, ...
      'Label', sprintf('Ort=%.1fm', mean(TDOA_error_m(:))));
xlabel('TDOA Hatası [m]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('TDOA Hata Dağılımı', 'FontWeight', 'bold');
grid on;

% --- Panel 4: Gerçek vs Ölçülen TDOA ---
subplot(2, 3, 4);
for tx = 1:4
    plot(1:N_pos, TDOA_true_m(:,tx), '--', 'LineWidth', 1.5, ...
         'DisplayName', sprintf('TX%d Gerçek', tx));
    hold on;
    plot(1:N_pos, TDOA_measured_m(:,tx), '.', 'MarkerSize', 12, ...
         'DisplayName', sprintf('TX%d Ölçülen', tx));
end
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('TDOA [m]', 'FontSize', 10);
title('Gerçek vs Ölçülen TDOA', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 7);
grid on;

% --- Panel 5: TX bazlı TDOA hatası ---
subplot(2, 3, 5);
tx_mean = mean(abs(TDOA_error_m), 1);
tx_std  = std(TDOA_error_m, 0, 1);
bar(tx_mean, 'FaceColor', [1 0.5 0]);
hold on;
errorbar(1:4, tx_mean, tx_std, 'k.', 'LineWidth', 2);
set(gca, 'XTickLabel', {'TX1','TX2','TX3','TX4'});
xlabel('Verici', 'FontSize', 10);
ylabel('Ort. |TDOA Hatası| [m]', 'FontSize', 10);
title('TX Bazlı TDOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 6: TDOA Hatası CDF ---
subplot(2, 3, 6);
err_abs  = abs(TDOA_error_m(:));
err_sort = sort(err_abs);
cdf_y    = (1:length(err_sort)) / length(err_sort) * 100;
plot(err_sort, cdf_y, 'b-', 'LineWidth', 2);
xline(mean(err_sort), 'r--', 'LineWidth', 1.5, ...
      'Label', sprintf('Ort=%.1fm', mean(err_sort)));
xlabel('|TDOA Hatası| [m]', 'FontSize', 10);
ylabel('Kümülatif Olasılık [%]', 'FontSize', 10);
title('TDOA Hatası CDF', 'FontWeight', 'bold');
grid on;
ylim([0, 100]);

sgtitle('ADIM 5: Diferansiyel TDOA Hesaplama', ...
        'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'step5_tdoa.png');
fprintf('✓ Grafik "step5_tdoa.png" kaydedildi.\n');

fprintf('\n========================================\n');
fprintf('  ADIM 5 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Sonraki adım: step6_wnls.m  (Konum Çözücü)\n');
fprintf('========================================\n');