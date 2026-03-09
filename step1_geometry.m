% =========================================================================
% ADIM 1: SİMÜLASYON GEOMETRİSİNİ KURMA
% =========================================================================
% Bu dosya çalıştırıldığında tüm koordinatlar ve UAV yörüngesi oluşturulur.
% Sonuçlar 'geometry_data.mat' dosyasına kaydedilir.
% =========================================================================

clear all;   % Önceki tüm değişkenleri temizle
close all;   % Önceki tüm grafikleri kapat
clc;         % Komut penceresini temizle

fprintf('========================================\n');
fprintf('  ADIM 1: Geometri Kurulumu Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: SİSTEM PARAMETRELERİ
% =========================================================================
% Bu parametreler tüm simülasyon boyunca sabit kalacak.
% Makaledeki değerlerle birebir aynı.

c = 3e8;          % Işık hızı [metre/saniye]
Fs = 2.048e6;     % DAB örnekleme frekansı [örnekleme/saniye]
                  % 2.048 MS/s = DAB Mode 1 standardı (ETSI EN 300 401)
h_uav = 200;      % UAV'ın sabit uçuş irtifası [metre]

fprintf('Sistem Parametreleri:\n');
fprintf('  Işık hızı     : %.2e m/s\n', c);
fprintf('  Örnekleme hızı: %.3f MS/s\n', Fs/1e6);
fprintf('  UAV irtifası  : %d m\n\n', h_uav);

% =========================================================================
% BÖLÜM 2: VERİCİ (TX) KONUMLARI
% =========================================================================
% 4 DAB vericisi 2km x 2km'lik alanın köşelerine yerleştiriliyor.
% Koordinatlar [x, y, z] formatında metre cinsindendir.
% z=0 yani vericiler yerde (anten kulesi yüksekliği ihmal edildi)
%
% Köşe düzeni (kuş bakışı):
%   TX3 (sol üst) -------- TX4 (sağ üst)
%       |                       |
%       |      2km x 2km        |
%       |                       |
%   TX1 (sol alt) -------- TX2 (sağ alt)

TX = [
     0,    0,   0;   % TX1: Sol-alt köşe  [x=0,    y=0,    z=0]
  2000,    0,   0;   % TX2: Sağ-alt köşe  [x=2000, y=0,    z=0]
     0, 2000,   0;   % TX3: Sol-üst köşe  [x=0,    y=2000, z=0]
  2000, 2000,   0;   % TX4: Sağ-üst köşe  [x=2000, y=2000, z=0]
];
% TX matrisi: 4 satır (4 verici) x 3 sütun (x, y, z koordinatı)

fprintf('Verici Konumları (TX) [metre]:\n');
fprintf('  %-5s  %-10s  %-10s  %-10s\n', 'TX#', 'X', 'Y', 'Z');
fprintf('  %-5s  %-10s  %-10s  %-10s\n', '---', '----------', '----------', '----------');
for i = 1:4
    fprintf('  TX%-3d  %-10.1f  %-10.1f  %-10.1f\n', i, TX(i,1), TX(i,2), TX(i,3));
end
fprintf('\n');

% =========================================================================
% BÖLÜM 3: REFERANS ALICI (Ref RX) KONUMU
% =========================================================================
% Referans alıcı: Konumu KESİNLİKLE bilinen sabit alıcı.
% Makaledeki gibi (1000, -500, 0) koordinatına yerleştiriyoruz.
% Bu alıcı UAV'ın konumunu hesaplamak için kullanılır.

Ref_RX = [1000, -500, 0];
% [x=1000m, y=-500m, z=0m] -> Alanın biraz dışında, güneye doğru

fprintf('Referans Alıcı (Ref RX) Konumu [metre]:\n');
fprintf('  X=%.1f, Y=%.1f, Z=%.1f\n\n', Ref_RX(1), Ref_RX(2), Ref_RX(3));

% =========================================================================
% BÖLÜM 4: UAV YÖRÜNGESI
% =========================================================================
% UAV dairesel bir yörüngede uçuyor.
% h=200m irtifada, alanın merkezinde.
%
% Daire parametreleri:
%   Merkez: (1000, 1000) -> 2km x 2km alanın tam ortası
%   Yarıçap: 500 metre
%   Yükseklik: h_uav = 200 metre (sabit)
%   Nokta sayısı: N_pos = 20 (yörünge üzerinde 20 konum)

N_pos = 20;             % Yörüngedeki toplam UAV konumu sayısı
radius = 500;           % Dairenin yarıçapı [metre]
center_x = 1000;        % Daire merkezi X koordinatı [metre]
center_y = 1000;        % Daire merkezi Y koordinatı [metre]

% linspace(0, 2*pi, N_pos): 0'dan 360 dereceye kadar N_pos eşit açı üret
% 2*pi = 360 derece (radyan cinsinden tam çember)
theta = linspace(0, 2*pi, N_pos + 1);  % N_pos+1 nokta üret
theta = theta(1:end-1);                 % Son nokta = ilk nokta olur, onu çıkar

% UAV koordinatlarını hesapla
UAV_true = zeros(N_pos, 3);            % N_pos x 3 boş matris oluştur
UAV_true(:, 1) = center_x + radius * cos(theta);   % X = merkez_x + r*cos(açı)
UAV_true(:, 2) = center_y + radius * sin(theta);   % Y = merkez_y + r*sin(açı)
UAV_true(:, 3) = h_uav * ones(N_pos, 1);           % Z = 200m (hep sabit)

fprintf('UAV Yörüngesi Parametreleri:\n');
fprintf('  Merkez      : (%.0f, %.0f) m\n', center_x, center_y);
fprintf('  Yarıçap     : %.0f m\n', radius);
fprintf('  İrtifa      : %.0f m\n', h_uav);
fprintf('  Nokta sayısı: %d\n\n', N_pos);

fprintf('UAV Yörünge Noktaları (ilk 5 tanesi):\n');
fprintf('  %-6s  %-12s  %-12s  %-12s\n', 'Konum', 'X [m]', 'Y [m]', 'Z [m]');
fprintf('  %-6s  %-12s  %-12s  %-12s\n', '------', '------------', '------------', '------------');
for i = 1:5
    fprintf('  %-6d  %-12.2f  %-12.2f  %-12.2f\n', i, UAV_true(i,1), UAV_true(i,2), UAV_true(i,3));
end
fprintf('  ... (toplam %d nokta)\n\n', N_pos);

% =========================================================================
% BÖLÜM 5: UZAKLIK MATRİSLERİNİ HESAPLA
% =========================================================================
% Konumlandırma denklemleri için her verici ile her alıcı arasındaki
% gerçek mesafeleri hesaplamamız gerekiyor.

% --- Referans Alıcı ile Vericiler Arası Mesafeler ---
% Her TX ile Ref_RX arasındaki 3D Öklid mesafesi
d_TX_RefRX = zeros(4, 1);   % 4 verici için 4 mesafe
for m = 1:4
    % 3D mesafe formülü: sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
    d_TX_RefRX(m) = sqrt(sum((TX(m,:) - Ref_RX).^2));
end

% --- UAV ile Vericiler Arası Mesafeler ---
% Her UAV konumu için her TX ile olan mesafe
% Sonuç: N_pos x 4 matris (20 konum x 4 verici)
d_TX_UAV = zeros(N_pos, 4);
for pos = 1:N_pos          % Her UAV konumu için
    for m = 1:4            % Her verici için
        d_TX_UAV(pos, m) = sqrt(sum((TX(m,:) - UAV_true(pos,:)).^2));
    end
end

fprintf('Verici - Referans Alıcı Mesafeleri:\n');
for m = 1:4
    fprintf('  TX%d -> Ref RX: %.2f m\n', m, d_TX_RefRX(m));
end
fprintf('\n');

fprintf('Verici - UAV Mesafeleri (Konum 1 için):\n');
for m = 1:4
    fprintf('  TX%d -> UAV[1]: %.2f m\n', m, d_TX_UAV(1,m));
end
fprintf('\n');

% =========================================================================
% BÖLÜM 6: GEOMETRİ KONTROLÜ (GDOP)
% =========================================================================
% GDOP (Geometric Dilution of Precision): Geometrinin konumlandırma
% doğruluğuna etkisini ölçer. Düşük GDOP = iyi geometri = daha doğru sonuç.
% Vericiler iyi dağılmışsa GDOP düşük olur.

fprintf('Geometri Kontrolü:\n');
% Alan boyutları
alan_genislik = max(TX(:,1)) - min(TX(:,1));
alan_yukseklik = max(TX(:,2)) - min(TX(:,2));
fprintf('  TX alanı: %.0f m x %.0f m\n', alan_genislik, alan_yukseklik);

% Ortalama mesafeler
fprintf('  Ort. TX-RefRX mesafesi: %.2f m\n', mean(d_TX_RefRX));
fprintf('  Ort. TX-UAV mesafesi  : %.2f m\n', mean(d_TX_UAV(:)));
fprintf('\n');

% =========================================================================
% BÖLÜM 7: VERİLERİ KAYDET
% =========================================================================
% Diğer adımlarda bu geometriyi tekrar kullanacağız.
% .mat dosyası MATLAB'ın kendi veri formatıdır.

save('geometry_data.mat', ...
    'TX', ...           % Verici koordinatları (4x3)
    'Ref_RX', ...       % Referans alıcı koordinatı (1x3)
    'UAV_true', ...     % Gerçek UAV yörüngesi (20x3)
    'N_pos', ...        % Yörünge nokta sayısı
    'd_TX_RefRX', ...   % TX-RefRX mesafeleri (4x1)
    'd_TX_UAV', ...     % TX-UAV mesafeleri (20x4)
    'c', ...            % Işık hızı
    'Fs', ...           % Örnekleme frekansı
    'h_uav', ...        % UAV irtifası
    'radius', ...       % Daire yarıçapı
    'center_x', ...     % Daire merkezi X
    'center_y' ...      % Daire merkezi Y
);

fprintf('✓ Veriler "geometry_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 8: 3D GRAFİK - GEOMETRİYİ GÖRSELLEŞTIR
% =========================================================================

figure('Name', 'Adım 1: Sistem Geometrisi', 'NumberTitle', 'off', ...
       'Position', [100, 100, 1000, 700]);

% --- Sol Panel: 3D Görünüm ---
subplot(1, 2, 1);
hold on;
grid on;
box on;

% Vericileri çiz (mavi kare)
scatter3(TX(:,1), TX(:,2), TX(:,3), 200, 'bs', ...
         'filled', 'DisplayName', 'DAB Vericiler (TX)');

% Verici etiketleri
for m = 1:4
    text(TX(m,1)+50, TX(m,2)+50, TX(m,3)+50, ...
         sprintf('TX%d', m), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
end

% Referans alıcıyı çiz (yeşil daire)
scatter3(Ref_RX(1), Ref_RX(2), Ref_RX(3), 200, 'g^', ...
         'filled', 'DisplayName', 'Referans Alıcı (Ref RX)');
text(Ref_RX(1)+50, Ref_RX(2)+50, Ref_RX(3)+50, ...
     'Ref RX', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'green');

% UAV yörüngesini çiz (kırmızı çizgi)
plot3(UAV_true(:,1), UAV_true(:,2), UAV_true(:,3), ...
      'r-', 'LineWidth', 2, 'DisplayName', 'UAV Yörüngesi');

% UAV noktalarını işaretle (kırmızı nokta)
scatter3(UAV_true(:,1), UAV_true(:,2), UAV_true(:,3), 50, 'ro', ...
         'filled', 'DisplayName', 'UAV Konumları');

% İlk UAV konumunu özel işaretle
scatter3(UAV_true(1,1), UAV_true(1,2), UAV_true(1,3), 150, 'r*', ...
         'DisplayName', 'UAV Başlangıç');

xlabel('X [metre]', 'FontSize', 12);
ylabel('Y [metre]', 'FontSize', 12);
zlabel('Z [metre]', 'FontSize', 12);
title('3D Sistem Geometrisi', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
view(45, 30);  % Görüş açısı: 45° yatay, 30° dikey

% --- Sağ Panel: Kuş Bakışı (2D) ---
subplot(1, 2, 2);
hold on;
grid on;
box on;

% Vericiler
scatter(TX(:,1), TX(:,2), 200, 'bs', 'filled', 'DisplayName', 'TX');
for m = 1:4
    text(TX(m,1)+60, TX(m,2)+60, sprintf('TX%d\n(%.0f,%.0f)', m, TX(m,1), TX(m,2)), ...
         'FontSize', 9, 'Color', 'blue');
end

% Referans alıcı
scatter(Ref_RX(1), Ref_RX(2), 200, 'g^', 'filled', 'DisplayName', 'Ref RX');
text(Ref_RX(1)+60, Ref_RX(2)+60, ...
     sprintf('Ref RX\n(%.0f,%.0f)', Ref_RX(1), Ref_RX(2)), ...
     'FontSize', 9, 'Color', 'green');

% UAV yörüngesi ve noktaları
plot(UAV_true(:,1), UAV_true(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'UAV Yörüngesi');
scatter(UAV_true(:,1), UAV_true(:,2), 50, 'ro', 'filled', 'DisplayName', 'UAV Konumları');

% Alan sınırları (TX'lerin oluşturduğu dikdörtgen)
rectangle('Position', [0, 0, 2000, 2000], 'EdgeColor', 'k', ...
          'LineStyle', '--', 'LineWidth', 1.5);

xlabel('X [metre]', 'FontSize', 12);
ylabel('Y [metre]', 'FontSize', 12);
title('Kuş Bakışı (2D Görünüm)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
axis equal;

% Genel başlık
sgtitle('ADIM 1: DAB SoOP TDOA Sistemi - Geometri', ...
        'FontSize', 16, 'FontWeight', 'bold');

% Grafiği kaydet
saveas(gcf, 'step1_geometry.png');
fprintf('✓ Grafik "step1_geometry.png" olarak kaydedildi.\n');

% =========================================================================
% ÖZET RAPOR
% =========================================================================
fprintf('\n========================================\n');
fprintf('  ADIM 1 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Oluşturulan değişkenler:\n');
fprintf('  TX       : %dx%d (verici koordinatları)\n', size(TX));
fprintf('  Ref_RX   : %dx%d (referans alıcı)\n', size(Ref_RX));
fprintf('  UAV_true : %dx%d (UAV yörüngesi)\n', size(UAV_true));
fprintf('  d_TX_RefRX: %dx%d (mesafe matrisi)\n', size(d_TX_RefRX));
fprintf('  d_TX_UAV : %dx%d (mesafe matrisi)\n', size(d_TX_UAV));
fprintf('\nSonraki adım: step2_dab_signal.m\n');
fprintf('========================================\n');