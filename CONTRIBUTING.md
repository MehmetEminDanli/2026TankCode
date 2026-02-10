Katkıda bulunma rehberi

Kısa yönergeler:

1. Repoyu klonlayın:
   git clone https://github.com/MehmetEminDanli/2026TankCode.git

2. Yeni bir branch oluşturun:
   git checkout -b feature/konu-adi

3. Değişiklikleri yapın, commit mesajları için format:
   type(scope): kısa-özet
   Örnek: feat(drive): add arcade smoothing

4. Push ve PR:
   git push -u origin feature/konu-adi
   GitHub üzerinde PR açıp uygun reviewer atayın.

Branch politikası:
- feature/* — yeni özellikler
- fix/* — hata düzeltmeleri
- chore/* — altyapı/ci döküman vb.

Kod stili:
- Java için proje ayarlarını takip edin. (IDE: IntelliJ/VSCode önerilir)

CI ve testler:
- Her PR için CI (Gradle build + test) çalışacaktır. CI ye takılan PR'lar merge edilemez.

İletişim:
- Reviewer veya repo sahibi ile GitHub üzerinden etiketleyerek iletişime geçin.
