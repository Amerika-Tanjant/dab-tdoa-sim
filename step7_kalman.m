% =========================================================================
% ADIM 7: KALMAN FİLTRESİ İLE KONUM İYİLEŞTİRME
% DAB SoOP TDOA Localization 
% =========================================================================
% Sabit Hızlı (Constant Velocity) Kalman Filtresi
%
% Durum vektörü: x = [px, py, pz, vx, vy, vz]'
%   px, py, pz : konum [metre]
%   vx, vy, vz : hız   [metre/adım]
%
% İki aşama:
%   1. Tahmin (Predict): Hareket modeline göre bir sonraki konumu tahmin et
%   2. Güncelleme (Update): Ölçümle tahmini birleştir
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 7: Kalman Filtresi Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('position_data.mat');

fprintf('✓ Veriler yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: KALMAN FİLTRESİ PARAMETRELERİ
% =========================================================================

dt = 1;   % Zaman adımı [adım başına saniye - normalize]
          % Gerçek sistemde: çerçeve süresi ~96ms olurdu

% --- Durum Geçiş Matrisi (F) ---
% Sabit hız modeli: konum = eski_konum + hız * dt
%
%        px   py   pz   vx   vy   vz
% px  [  1    0    0    dt   0    0  ]
% py  [  0    1    0    0    dt   0  ]
% pz  [  0    0    1    0    0    dt ]
% vx  [  0    0    0    1    0    0  ]
% vy  [  0    0    0    0    1    0  ]
% vz  [  0    0    0    0    0    1  ]

F = [1 0 0 dt 0  0;
     0 1 0 0  dt 0;
     0 0 1 0  0  dt;
     0 0 0 1  0  0;
     0 0 0 0  1  0;
     0 0 0 0  0  1];

% --- Gözlem Matrisi (H) ---
% Sadece konumu ölçüyoruz (hızı ölçemiyoruz)
% Ölçüm: z = [px, py, pz]
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

% --- Proses Gürültüsü Kovaryansı (Q) ---
% UAV'ın hareketi ne kadar tahmin edilemez?
% Büyük Q → filtreye daha az güven, ölçüme daha çok güven
% Küçük Q → filtreye daha çok güven, pürüzsüz ama yavaş tepki
sigma_a = 2.0;   % İvme belirsizliği [m/adım²]

% Sürekli zamanlı proses gürültüsü modeli
Q = sigma_a^2 * ...
    [dt^4/4  0       0       dt^3/2  0       0;
     0       dt^4/4  0       0       dt^3/2  0;
     0       0       dt^4/4  0       0       dt^3/2;
     dt^3/2  0       0       dt^2    0       0;
     0       dt^3/2  0       0       dt^2    0;
     0       0       dt^3/2  0       0       dt^2];

% --- Ölçüm Gürültüsü Kovaryansı (R) ---
% WNLS'den gelen konum ölçümü ne kadar gürültülü?
% mean_err değerini kullan (Adım 6'dan)
sigma_meas = mean_err;   % ~143m

R = sigma_meas^2 * eye(3);   % 3x3 köşegen matris

fprintf('Kalman Filtresi Parametreleri:\n');
fprintf('  Proses gürültüsü (σ_a)   : %.1f m/adım²\n', sigma_a);
fprintf('  Ölçüm gürültüsü (σ_meas) : %.1f m\n', sigma_meas);
fprintf('  Durum boyutu             : %d (konum + hız)\n', size(F,1));
fprintf('  Ölçüm boyutu             : %d (sadece konum)\n\n', size(H,1));

% =========================================================================
% BÖLÜM 3: KALMAN FİLTRESİ BAŞLANGIÇ KOŞULLARI
% =========================================================================

% Başlangıç durum tahmini: ilk WNLS ölçümünü kullan
x_init = [UAV_estimated(1,1);   % px
          UAV_estimated(1,2);   % py
          UAV_estimated(1,3);   % pz
          0;                    % vx (başta hız bilinmiyor → 0)
          0;                    % vy
          0];                   % vz

% Başlangıç hata kovaryansı: büyük belirsizlik
P_init = diag([sigma_meas^2, sigma_meas^2, sigma_meas^2, ...
               100^2, 100^2, 100^2]);
% İlk 3: konum belirsizliği (WNLS hatası kadar)
% Son 3: hız belirsizliği (tamamen bilinmiyor)

% =========================================================================
% BÖLÜM 4: KALMAN FİLTRESİ DÖNGÜSÜ
% =========================================================================

fprintf('Kalman filtresi çalışıyor...\n');

% Sonuçları saklayacak matrisler
x_filtered    = zeros(6, N_pos);   % Filtre çıkışı (durum)
P_filtered    = zeros(6, 6, N_pos); % Hata kovaryansı
innovation    = zeros(3, N_pos);    % Yenilik (innovation = z - H*x_pred)
K_gains       = zeros(6, 3, N_pos); % Kalman kazancı

% Başlat
x_k = x_init;
P_k = P_init;

for k = 1:N_pos

    % ==================
    % TAHMİN ADIMI (Predict)
    % ==================
    % Durum tahmini: bir adım ileriye projeksiyon
    x_pred = F * x_k;          % x_{k|k-1} = F * x_{k-1|k-1}

    % Kovaryans tahmini
    P_pred = F * P_k * F' + Q; % P_{k|k-1} = F*P*F' + Q

    % ==================
    % GÜNCELLEME ADIMI (Update)
    % ==================
    % Ölçüm: WNLS'den gelen konum
    z_k = UAV_estimated(k, :)';   % 3x1 ölçüm vektörü

    % Yenilik (Innovation): Ölçüm ile tahmin arasındaki fark
    y_k = z_k - H * x_pred;       % z - H*x_pred
    innovation(:, k) = y_k;

    % Yenilik kovaryansı
    S_k = H * P_pred * H' + R;    % H*P*H' + R

    % Kalman Kazancı: tahmine mi ölçüme mi daha çok güvenilecek?
    % Büyük K → ölçüme daha çok güven
    % Küçük K → tahmine daha çok güven
    K_k = P_pred * H' / S_k;      % P*H' * S^(-1)
    K_gains(:, :, k) = K_k;

    % Durum güncellemesi
    x_k = x_pred + K_k * y_k;     % x_{k|k} = x_{k|k-1} + K*y

    % Kovaryans güncellemesi (Joseph formu - sayısal kararlılık için)
    I_KH = eye(6) - K_k * H;
    P_k  = I_KH * P_pred * I_KH' + K_k * R * K_k';

    % Kaydet
    x_filtered(:, k)    = x_k;
    P_filtered(:, :, k) = P_k;
end

fprintf('✓ Kalman filtresi tamamlandı.\n\n');

% Filtrelenmiş konumları çıkar
UAV_filtered = x_filtered(1:3, :)';   % N_pos x 3

% =========================================================================
% BÖLÜM 5: HATA ANALİZİ - KALMAN vs WNLS KARŞILAŞTIRMASI
% =========================================================================

pos_error_kalman = zeros(N_pos, 1);
pos_error_wnls   = zeros(N_pos, 1);

for k = 1:N_pos
    pos_error_kalman(k) = norm(UAV_filtered(k,:) - UAV_true(k,:));
    pos_error_wnls(k)   = norm(UAV_estimated(k,:) - UAV_true(k,:));
end

rmse_kalman = sqrt(mean(pos_error_kalman.^2));
rmse_wnls   = sqrt(mean(pos_error_wnls.^2));
mean_kalman = mean(pos_error_kalman);
mean_wnls   = mean(pos_error_wnls);
p95_kalman  = prctile(pos_error_kalman, 95);
p95_wnls    = prctile(pos_error_wnls, 95);

fprintf('=== WNLS vs Kalman Karşılaştırması ===\n');
fprintf('  %-25s  %-12s  %-12s\n', 'Metrik', 'WNLS', 'Kalman');
fprintf('  %-25s  %-12s  %-12s\n', '-------------------------', '------------', '------------');
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'Ortalama Hata [m]', mean_wnls,   mean_kalman);
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'RMSE [m]',          rmse_wnls,   rmse_kalman);
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'P95 Hata [m]',      p95_wnls,    p95_kalman);
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'Maks Hata [m]', ...
        max(pos_error_wnls), max(pos_error_kalman));
fprintf('\n');

iyilesme = (mean_wnls - mean_kalman) / mean_wnls * 100;
fprintf('  Kalman iyileşmesi: %.1f%%\n\n', iyilesme);

% =========================================================================
% BÖLÜM 6: KAYDET
% =========================================================================

save('kalman_data.mat', ...
    'UAV_filtered', ...
    'x_filtered', ...
    'P_filtered', ...
    'pos_error_kalman', ...
    'pos_error_wnls', ...
    'rmse_kalman', ...
    'rmse_wnls', ...
    'mean_kalman', ...
    'mean_wnls', ...
    'p95_kalman', ...
    'p95_wnls', ...
    'innovation', ...
    'K_gains');

fprintf('✓ Veriler "kalman_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 7: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 7: Kalman Filtresi', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1200, 800]);

% --- Panel 1: 2D Yörünge Karşılaştırması ---
subplot(2, 3, [1, 2]);
hold on; grid on;

% TX ve Ref RX
scatter(TX(:,1), TX(:,2), 120, 'bs', 'filled', 'HandleVisibility', 'off');
scatter(Ref_RX(1), Ref_RX(2), 120, 'g^', 'filled', 'HandleVisibility', 'off');

% Yörüngeler
plot(UAV_true(:,1),     UAV_true(:,2),     'k-',  'LineWidth', 2.5, ...
     'DisplayName', 'Gerçek');
plot(UAV_estimated(:,1),UAV_estimated(:,2),'r--', 'LineWidth', 1.8, ...
     'DisplayName', sprintf('WNLS (%.0fm)', mean_wnls));
plot(UAV_filtered(:,1), UAV_filtered(:,2), 'b-',  'LineWidth', 2, ...
     'DisplayName', sprintf('Kalman (%.0fm)', mean_kalman));

% Noktalar
scatter(UAV_true(:,1),     UAV_true(:,2),     50, 'ko', 'filled', 'HandleVisibility','off');
scatter(UAV_estimated(:,1),UAV_estimated(:,2),50, 'rv', 'filled', 'HandleVisibility','off');
scatter(UAV_filtered(:,1), UAV_filtered(:,2), 50, 'b^', 'filled', 'HandleVisibility','off');

xlabel('X [m]', 'FontSize', 11);
ylabel('Y [m]', 'FontSize', 11);
title('Yörünge Karşılaştırması: Gerçek vs WNLS vs Kalman', ...
      'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
axis equal;
xlim([-200, 2200]);
ylim([-700, 2700]);

% --- Panel 3: Hata Histogramı Karşılaştırması ---
subplot(2, 3, 3);
hold on;
histogram(pos_error_wnls,   12, 'FaceColor', 'red',  'FaceAlpha', 0.5, ...
          'DisplayName', sprintf('WNLS (%.0fm)', mean_wnls));
histogram(pos_error_kalman, 12, 'FaceColor', 'blue', 'FaceAlpha', 0.5, ...
          'DisplayName', sprintf('Kalman (%.0fm)', mean_kalman));
xlabel('Konum Hatası [m]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('Hata Dağılımı: WNLS vs Kalman', 'FontWeight', 'bold');
legend('FontSize', 9);
grid on;

% --- Panel 4: Zaman Serisi Hata Karşılaştırması ---
subplot(2, 3, 4);
plot(1:N_pos, pos_error_wnls,   'r-o', 'LineWidth', 1.8, 'MarkerSize', 6, ...
     'DisplayName', sprintf('WNLS (%.0fm)', mean_wnls));
hold on;
plot(1:N_pos, pos_error_kalman, 'b-s', 'LineWidth', 2,   'MarkerSize', 6, ...
     'DisplayName', sprintf('Kalman (%.0fm)', mean_kalman));
yline(mean_wnls,   'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(mean_kalman, 'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('3D Konum Hatası [m]', 'FontSize', 10);
title('Hata Zaman Serisi', 'FontWeight', 'bold');
legend('FontSize', 9, 'Location', 'northeast');
grid on;
xlim([1, N_pos]);

% --- Panel 5: CDF Karşılaştırması ---
subplot(2, 3, 5);
err_wnls_s   = sort(pos_error_wnls);
err_kalm_s   = sort(pos_error_kalman);
cdf_y        = (1:N_pos) / N_pos * 100;
plot(err_wnls_s, cdf_y, 'r-', 'LineWidth', 2.5, ...
     'DisplayName', sprintf('WNLS (P95=%.0fm)', p95_wnls));
hold on;
plot(err_kalm_s, cdf_y, 'b-', 'LineWidth', 2.5, ...
     'DisplayName', sprintf('Kalman (P95=%.0fm)', p95_kalman));
yline(95, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('Konum Hatası [m]', 'FontSize', 10);
ylabel('Kümülatif Olasılık [%]', 'FontSize', 10);
title('CDF Karşılaştırması', 'FontWeight', 'bold');
legend('FontSize', 9, 'Location', 'southeast');
grid on;
ylim([0, 100]);

% --- Panel 6: Kalman Kazancı (K) ---
subplot(2, 3, 6);
K_px = squeeze(K_gains(1, 1, :));   % px için Kalman kazancı
K_py = squeeze(K_gains(2, 2, :));   % py için
K_pz = squeeze(K_gains(3, 3, :));   % pz için
plot(1:N_pos, K_px, 'r-o', 'LineWidth', 1.8, 'MarkerSize', 5, 'DisplayName', 'K_{px}');
hold on;
plot(1:N_pos, K_py, 'b-s', 'LineWidth', 1.8, 'MarkerSize', 5, 'DisplayName', 'K_{py}');
plot(1:N_pos, K_pz, 'g-^', 'LineWidth', 1.8, 'MarkerSize', 5, 'DisplayName', 'K_{pz}');
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('Kalman Kazancı', 'FontSize', 10);
title('Kalman Kazancının Yakınsamas', 'FontWeight', 'bold');
legend('FontSize', 9);
grid on;
xlim([1, N_pos]);

sgtitle(sprintf('ADIM 7: Kalman Filtresi | WNLS: %.1fm → Kalman: %.1fm (%.0f%% iyileşme)', ...
                mean_wnls, mean_kalman, iyilesme), ...
        'FontSize', 12, 'FontWeight', 'bold');

saveas(gcf, 'step7_kalman.png');
fprintf('✓ Grafik "step7_kalman.png" kaydedildi.\n');

fprintf('\n========================================\n');
fprintf('  ADIM 7 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Sonraki adım: step8_analysis.m\n');
fprintf('========================================\n');