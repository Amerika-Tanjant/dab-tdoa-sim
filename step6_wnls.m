% =========================================================================
% ADIM 6: KONUM ÇÖZÜCÜ (WNLS - Weighted Nonlinear Least Squares)
% DAB SoOP TDOA Localization 
% =========================================================================
% TDOA değerlerini [x, y, z] koordinatına dönüştürme.
%
% Yöntem: Gauss-Newton iterasyonu ile WNLS
%   1. Başlangıç noktası belirle (alanın merkezi)
%   2. Artıkları (residuals) hesapla: ölçülen - tahmin edilen TDOA
%   3. Jacobian matrisini oluştur
%   4. Ağırlıklı güncelleme: Δx = (J'WJ)^(-1) J'W r
%   5. Yakınsayana kadar tekrarla
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 6: Konum Çözücü Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('tdoa_data.mat');

fprintf('✓ Tüm veriler yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: WNLS ÇÖZÜCÜ FONKSİYONU
% =========================================================================
% Gauss-Newton iterasyonları ile TDOA → konum dönüşümü
%
% Girdi:
%   tdoa_vec : [4x1] - 4 TX için TDOA ölçümleri [metre]
%   TX       : [4x3] - Verici koordinatları
%   Ref_RX   : [1x3] - Referans alıcı koordinatı
%   x0       : [3x1] - Başlangıç tahmini [x, y, z]
%   W        : [4x4] - Ağırlık matrisi (gürültü kovaryansının tersi)
%
% Çıktı:
%   x_est    : [3x1] - Tahmin edilen UAV konumu [x, y, z]
%   converged: Yakınsadı mı?

    function [x_est, converged, n_iter] = wnls_solver(tdoa_vec, TX_pos, RefRX, x0, W)
        MAX_ITER = 50;       % Maksimum iterasyon sayısı
        TOL      = 1e-3;     % Yakınsama toleransı [metre]

        x_cur    = x0(:);    % Mevcut tahmin (sütun vektör)
        converged = false;
        n_iter   = 0;

        for iter = 1:MAX_ITER
            n_iter = iter;

            % --- Artık vektörü hesapla (residuals) ---
            % r_i = TDOA_ölçülen_i - TDOA_tahmin_i
            % TDOA_tahmin_i = d(x,TX_i) - d(RefRX,TX_i)
            r = zeros(4, 1);
            for i = 1:4
                d_uav_tx  = norm(x_cur - TX_pos(i,:)');   % UAV-TX mesafesi
                d_ref_tx  = norm(RefRX' - TX_pos(i,:)');  % RefRX-TX mesafesi
                tdoa_pred = d_uav_tx - d_ref_tx;           % Tahmin edilen TDOA
                r(i)      = tdoa_vec(i) - tdoa_pred;       % Artık
            end

            % --- Jacobian matrisi (J) ---
            % J_ij = ∂(TDOA_i) / ∂(x_j)
            % = (x_cur - TX_i) / d(x_cur, TX_i)  →  birim yön vektörü
            J = zeros(4, 3);
            for i = 1:4
                d_uav_tx = norm(x_cur - TX_pos(i,:)');
                if d_uav_tx < 1e-6; d_uav_tx = 1e-6; end  % Sıfıra bölmeyi önle
                J(i, :) = (x_cur - TX_pos(i,:)') / d_uav_tx;
            end

            % --- Ağırlıklı Normal Denklem ---
            % Δx = (J'WJ)^(-1) J'W r
            JtW    = J' * W;
            JtWJ   = JtW * J;

            % Matris tekil mi kontrol et
            if rcond(JtWJ) < 1e-12
                break;  % Tekil matris, dur
            end

            delta_x = JtWJ \ (JtW * r);   % Konum güncellemesi

            % --- Güncelle ---
            x_cur = x_cur + delta_x;

            % --- Yakınsama kontrolü ---
            if norm(delta_x) < TOL
                converged = true;
                break;
            end
        end

        x_est = x_cur;
    end

% =========================================================================
% BÖLÜM 3: AĞIRLIK MATRİSİ
% =========================================================================
% W = ölçüm gürültüsünün tersine orantılı
% Basit başlangıç: tüm TX'lere eşit ağırlık (birim matris)
% İleride: SNR veya mesafeye göre adaptif ağırlık kullanılabilir

W = eye(4);   % 4x4 birim matris → eşit ağırlık

% =========================================================================
% BÖLÜM 4: BAŞLANGIÇ TAHMİNİ
% =========================================================================
% İyi başlangıç noktası = daha az iterasyon ve daha iyi yakınsama
% TX'lerin merkezi mantıklı bir başlangıç noktasıdır

x0 = [mean(TX(:,1)); mean(TX(:,2)); 200];   % [1000; 1000; 200]

fprintf('Başlangıç noktası: [%.0f, %.0f, %.0f] m\n\n', x0);

% =========================================================================
% BÖLÜM 5: TÜM UAV KONUMLARI İÇİN KONUM ÇÖZÜMÜ
% =========================================================================

fprintf('WNLS konum çözümü hesaplanıyor...\n');

UAV_estimated  = zeros(N_pos, 3);  % Tahmin edilen konumlar
converge_flags = zeros(N_pos, 1);  % Yakınsama durumu
iter_counts    = zeros(N_pos, 1);  % İterasyon sayıları

for pos = 1:N_pos
    % Bu konum için TDOA vektörü [metre]
    tdoa_vec = TDOA_measured_m(pos, :)';   % 4x1 sütun vektör

    % WNLS çöz
    [x_est, conv, n_iter] = wnls_solver(tdoa_vec, TX, Ref_RX, x0, W);

    UAV_estimated(pos, :)  = x_est';
    converge_flags(pos)    = conv;
    iter_counts(pos)       = n_iter;
end

fprintf('✓ Tüm konumlar çözüldü.\n\n');

% =========================================================================
% BÖLÜM 6: HATA ANALİZİ
% =========================================================================

% 3D konum hatası: sqrt((x_est-x_true)^2 + (y_est-y_true)^2 + (z_est-z_true)^2)
pos_error_3d = zeros(N_pos, 1);
pos_error_2d = zeros(N_pos, 1);   % Sadece yatay hata (operasyonel açıdan önemli)

for pos = 1:N_pos
    diff_3d        = UAV_estimated(pos,:) - UAV_true(pos,:);
    pos_error_3d(pos) = norm(diff_3d);
    pos_error_2d(pos) = norm(diff_3d(1:2));   % Sadece X-Y
end

% İstatistikler
rmse_3d   = sqrt(mean(pos_error_3d.^2));
rmse_2d   = sqrt(mean(pos_error_2d.^2));
mean_err  = mean(pos_error_3d);
p95_err   = prctile(pos_error_3d, 95);
conv_rate = sum(converge_flags) / N_pos * 100;

fprintf('=== WNLS Konum Çözücü Sonuçları ===\n');
fprintf('  RMSE (3D)          : %.2f m\n', rmse_3d);
fprintf('  RMSE (2D yatay)    : %.2f m\n', rmse_2d);
fprintf('  Ortalama hata      : %.2f m\n', mean_err);
fprintf('  95. yüzdelik hata  : %.2f m\n', p95_err);
fprintf('  Yakınsama oranı    : %.1f%%\n', conv_rate);
fprintf('  Ort. iterasyon     : %.1f\n\n', mean(iter_counts));

fprintf('Hedef (makaleden)    : ~37.14 m ortalama hata\n');
fprintf('Mevcut sonuç         : %.2f m\n\n', mean_err);

% =========================================================================
% BÖLÜM 7: KAYDET
% =========================================================================

save('position_data.mat', ...
    'UAV_estimated', ...    % Tahmin edilen konumlar    [N_pos x 3]
    'pos_error_3d', ...     % 3D konum hatası          [N_pos x 1]
    'pos_error_2d', ...     % 2D yatay konum hatası    [N_pos x 1]
    'rmse_3d', ...
    'rmse_2d', ...
    'mean_err', ...
    'p95_err', ...
    'conv_rate', ...
    'iter_counts', ...
    'converge_flags');

fprintf('✓ Veriler "position_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 8: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 6: WNLS Konum Çözücü', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1200, 800]);

% --- Panel 1: Gerçek vs Tahmin Yörüngesi (2D) ---
subplot(2, 3, [1, 2]);
hold on; grid on;

% TX'ler
scatter(TX(:,1), TX(:,2), 150, 'bs', 'filled');
for m = 1:4
    text(TX(m,1)+60, TX(m,2)+60, sprintf('TX%d',m), ...
         'FontSize', 9, 'Color', 'blue', 'FontWeight', 'bold');
end

% Ref RX
scatter(Ref_RX(1), Ref_RX(2), 150, 'g^', 'filled');
text(Ref_RX(1)+60, Ref_RX(2)+60, 'Ref RX', ...
     'FontSize', 9, 'Color', 'green', 'FontWeight', 'bold');

% Gerçek yörünge
plot(UAV_true(:,1), UAV_true(:,2), 'k-', 'LineWidth', 2, ...
     'DisplayName', 'Gerçek Yörünge');
scatter(UAV_true(:,1), UAV_true(:,2), 60, 'ko', 'filled', ...
        'DisplayName', 'Gerçek Konumlar');

% Tahmin edilen yörünge
plot(UAV_estimated(:,1), UAV_estimated(:,2), 'r--', 'LineWidth', 2, ...
     'DisplayName', 'WNLS Tahmini');
scatter(UAV_estimated(:,1), UAV_estimated(:,2), 60, 'r^', 'filled', ...
        'DisplayName', 'Tahmin Konumları');

% Hata çizgileri (gerçek → tahmin)
for pos = 1:N_pos
    plot([UAV_true(pos,1), UAV_estimated(pos,1)], ...
         [UAV_true(pos,2), UAV_estimated(pos,2)], ...
         'r-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

xlabel('X [m]', 'FontSize', 11);
ylabel('Y [m]', 'FontSize', 11);
title('Gerçek vs WNLS Tahmin Yörüngesi (Kuş Bakışı)', ...
      'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
axis equal;
xlim([-200, 2200]);
ylim([-700, 2700]);

% --- Panel 3: Konum Hata Histogramı ---
subplot(2, 3, 3);
histogram(pos_error_3d, 12, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'white');
xline(mean_err, 'b--', 'LineWidth', 2, ...
      'Label', sprintf('Ort=%.1fm', mean_err));
xline(p95_err,  'g--', 'LineWidth', 2, ...
      'Label', sprintf('P95=%.1fm', p95_err));
xlabel('3D Konum Hatası [m]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('Konum Hata Dağılımı', 'FontWeight', 'bold');
grid on;

% --- Panel 4: Hata Zaman Serisi ---
subplot(2, 3, 4);
plot(1:N_pos, pos_error_3d, 'r-o', 'LineWidth', 2, 'MarkerSize', 7, ...
     'DisplayName', '3D Hata');
hold on;
plot(1:N_pos, pos_error_2d, 'b-s', 'LineWidth', 2, 'MarkerSize', 7, ...
     'DisplayName', '2D Yatay Hata');
yline(mean_err, 'r--', 'LineWidth', 1.5, ...
      'Label', sprintf('Ort=%.1fm', mean_err));
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('Konum Hatası [m]', 'FontSize', 10);
title('Konum Hatası - Her UAV Noktası', 'FontWeight', 'bold');
legend('FontSize', 9);
grid on;
xlim([1, N_pos]);

% --- Panel 5: CDF ---
subplot(2, 3, 5);
err_sort = sort(pos_error_3d);
cdf_y    = (1:N_pos) / N_pos * 100;
plot(err_sort, cdf_y, 'r-', 'LineWidth', 2.5);
xline(mean_err, 'b--', 'LineWidth', 1.5, ...
      'Label', sprintf('Ort=%.1fm', mean_err));
xline(p95_err,  'g--', 'LineWidth', 1.5, ...
      'Label', sprintf('P95=%.1fm', p95_err));
xlabel('3D Konum Hatası [m]', 'FontSize', 10);
ylabel('Kümülatif Olasılık [%]', 'FontSize', 10);
title('Konum Hatası CDF', 'FontWeight', 'bold');
grid on;
ylim([0, 100]);

% --- Panel 6: İterasyon Sayısı ---
subplot(2, 3, 6);
bar(1:N_pos, iter_counts, 'FaceColor', [0.5 0.7 0.9]);
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('İterasyon Sayısı', 'FontSize', 10);
title('WNLS İterasyon Sayısı', 'FontWeight', 'bold');
yline(mean(iter_counts), 'r--', 'LineWidth', 2, ...
      'Label', sprintf('Ort=%.1f', mean(iter_counts)));
grid on;

sgtitle(sprintf('ADIM 6: WNLS Konum Çözücü | Ort. Hata: %.2f m | P95: %.2f m', ...
                mean_err, p95_err), ...
        'FontSize', 13, 'FontWeight', 'bold');

saveas(gcf, 'step6_wnls.png');
fprintf('✓ Grafik "step6_wnls.png" kaydedildi.\n');

fprintf('\n========================================\n');
fprintf('  ADIM 6 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Sonraki adım: step7_kalman.m\n');
fprintf('========================================\n');