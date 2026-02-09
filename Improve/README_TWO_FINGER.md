# Two Finger Tracking - Dual Arduino System

Program ini menggunakan MediaPipe dan OpenCV untuk melacak gerakan jari telunjuk tangan kiri dan kanan secara bersamaan, kemudian mengirim data sudut servo ke 2 Arduino terpisah.

## 📋 Daftar File

1. **two_finger_track.py** - Program Python utama untuk tracking
2. **servo_left.ino** - Program Arduino untuk tangan kiri
3. **servo_right.ino** - Program Arduino untuk tangan kanan

## 🔧 Hardware Setup

### Arduino Kiri (Left Hand)
- **Port**: COM3 (default, bisa diubah di Python)
- **Pin Servo**: Pin 9
- **Fungsi**: Kontrol jari telunjuk tangan kiri

### Arduino Kanan (Right Hand)
- **Port**: COM4 (default, bisa diubah di Python)
- **Pin Servo**: Pin 9
- **Fungsi**: Kontrol jari telunjuk tangan kanan

### Koneksi Servo
Untuk setiap Arduino:
- **Servo Signal** → Pin 9
- **Servo VCC** → 5V
- **Servo GND** → GND

⚠️ **Penting**: Jika menggunakan banyak servo, gunakan power supply eksternal untuk servo (jangan langsung dari Arduino).

## 📦 Instalasi Software

### 1. Install Dependencies Python
```bash
pip install opencv-python mediapipe pyserial numpy
```

### 2. Upload Program ke Arduino
1. Buka Arduino IDE
2. Upload **servo_left.ino** ke Arduino pertama (akan menjadi COM3)
3. Upload **servo_right.ino** ke Arduino kedua (akan menjadi COM4)
4. Catat port COM yang terdeteksi untuk masing-masing Arduino

### 3. Konfigurasi Port COM
Edit file **two_finger_track.py** pada baris 22-23:
```python
SERIAL_PORT_LEFT = "COM3"    # Sesuaikan dengan Arduino kiri
SERIAL_PORT_RIGHT = "COM4"   # Sesuaikan dengan Arduino kanan
```

## 🚀 Cara Menggunakan

### 1. Jalankan Program
```bash
python two_finger_track.py
```

### 2. Kalibrasi
Program perlu dikalibrasi untuk setiap tangan:

**Untuk Tangan Kiri:**
1. Buka jari telunjuk kiri selebar-lebarnya
2. Tekan tombol **O** atau **[**
3. Tutup jari telunjuk kiri serapat-rapatnya
4. Tekan tombol **C** atau **]**

**Untuk Tangan Kanan:**
1. Buka jari telunjuk kanan selebar-lebarnya
2. Tekan tombol **O** atau **[**
3. Tutup jari telunjuk kanan serapat-rapatnya
4. Tekan tombol **C** atau **]**

### 3. Kontrol
Setelah kalibrasi, gerakan jari telunjuk Anda akan otomatis terdeteksi dan dikirim ke servo masing-masing:
- **Jari terbuka** → Servo 0° (OPEN)
- **Jari tertutup** → Servo 180° (CLOSE)

## ⌨️ Keyboard Controls

| Tombol | Fungsi |
|--------|--------|
| **O** atau **[** | Simpan posisi OPEN (jari terbuka) |
| **C** atau **]** | Simpan posisi CLOSE (jari tertutup) |
| **R** | Reset kalibrasi |
| **P** | Pause/Resume |
| **Q** | Quit (keluar program) |

## 📊 Status Indikator

### Di Layar:
- **Blue Box**: Tangan kiri terdeteksi
- **Orange Box**: Tangan kanan terdeteksi
- **Blue Bar**: Posisi servo tangan kiri (0-180°)
- **Orange Bar**: Posisi servo tangan kanan (0-180°)
- **"Left:CAL"**: Tangan kiri sudah dikalibrasi
- **"Right:CAL"**: Tangan kanan sudah dikalibrasi

### Di Serial Monitor Arduino:
- **LEFT HAND**: `READY - LEFT HAND Index Finger`
- **RIGHT HAND**: `READY - RIGHT HAND Index Finger`

## 🔍 Troubleshooting

### Program tidak bisa connect ke Arduino
1. Cek Device Manager untuk melihat port COM yang benar
2. Pastikan tidak ada program lain yang menggunakan port serial
3. Tutup Arduino IDE Serial Monitor sebelum menjalankan Python
4. Update driver CH340/CH341 jika menggunakan Arduino clone

### Kamera tidak terdeteksi
- Ubah `CAM_INDEX = 0` menjadi `CAM_INDEX = 1` atau `2` di Python

### Servo bergerak terbalik
- Set `INVERT = true` di file Arduino (.ino)

### Tracking tidak akurat
- Pastikan pencahayaan cukup
- Kalibrasi ulang dengan tekan **R** kemudian ulangi proses kalibrasi
- Jaga jarak kamera 30-60 cm dari tangan

### Data tidak terkirim ke servo
- Pastikan kabel USB terhubung dengan baik
- Cek baud rate (harus 115200 di Python dan Arduino)
- Restart Arduino dan program Python

## ⚙️ Kustomisasi

### Mengubah Pin Servo
Edit di file Arduino (.ino):
```cpp
#define SERVO_PIN 9  // Ubah ke pin yang diinginkan
```

### Mengubah Smoothing
Edit di file Arduino (.ino):
```cpp
const uint8_t SMOOTH_STEP = 3;  // 1=halus lambat, 10=cepat kasar
```

### Mengubah Range Servo
Edit di file Arduino (.ino):
```cpp
const uint8_t MIN_LIMIT = 0;    // Batas minimum
const uint8_t MAX_LIMIT = 180;  // Batas maximum
```

### Mengubah FPS Pengiriman Data
Edit di file Python:
```python
SERIAL_SEND_EVERY_MS = 33  # 33ms ≈ 30Hz, ubah sesuai kebutuhan
```

## 📝 Format Data Serial

### Python → Arduino Kiri:
```
L,<angle>\n
```
Contoh: `L,90\n` (gerakkan servo kiri ke 90°)

### Python → Arduino Kanan:
```
R,<angle>\n
```
Contoh: `R,45\n` (gerakkan servo kanan ke 45°)

## 🎯 Tips Penggunaan

1. **Pencahayaan**: Gunakan pencahayaan yang cukup untuk tracking yang akurat
2. **Background**: Background yang kontras dengan warna kulit akan lebih baik
3. **Jarak**: Jaga jarak 30-60 cm dari kamera
4. **Kalibrasi**: Lakukan kalibrasi setiap kali mulai menggunakan
5. **Power Servo**: Gunakan power supply eksternal untuk hasil yang lebih stabil

## 📄 Lisensi

Program ini dibuat untuk keperluan edukasi dan riset.

## 🤝 Kontribusi

Silakan modifikasi dan kembangkan sesuai kebutuhan Anda!
