# Robot Transporter Berbasis Inverse dan Forward Kinematik

Program Arduino/ESP32 untuk robot transporter differential drive yang menggunakan forward kinematics, inverse kinematics, encoder, kontrol proporsional, LCD I2C, keypad, servo gripper, dan mekanisme lift.

Robot dapat menerima target koordinat dari keypad, bergerak menuju target, menjepit dan mengangkat barang, lalu kembali ke posisi awal.

## Fitur Utama

- Robot differential drive berbasis ESP32.
- Dua strategi gerak:
  - **Mode 1 - Kurva:** robot langsung bergerak menuju titik target dengan koreksi arah selama berjalan.
  - **Mode 2 - Rotasi + Maju:** robot berputar menghadap target terlebih dahulu, lalu bergerak maju lurus.
- Input koordinat target X dan Y melalui keypad dalam satuan sentimeter.
- Tampilan status melalui LCD I2C 16x2.
- Pembacaan encoder roda kiri dan kanan untuk odometry.
- Perhitungan forward kinematics untuk estimasi posisi robot.
- Perhitungan inverse kinematics untuk menentukan kecepatan roda kiri dan kanan.
- Kontrol proporsional untuk kecepatan linear, angular, dan roda.
- Servo gripper untuk menjepit/melepas barang.
- Motor lift untuk menaikkan/menurunkan barang.

## Perangkat Keras

Komponen yang digunakan:

- ESP32
- Driver motor DC
- 2 motor DC untuk roda kiri dan kanan
- 2 encoder roda
- LCD I2C 16x2
- Keypad 4x3
- Servo gripper
- Motor lift atau aktuator pengangkat
- Rangka robot differential drive
- Catu daya sesuai kebutuhan motor dan ESP32

## Library Arduino

Pastikan library berikut sudah terpasang di Arduino IDE:

- `Wire`
- `LiquidCrystal_I2C`
- `Keypad`
- `ESP32Servo`
- `math.h`

Library `Wire` dan `math.h` biasanya sudah tersedia secara default. Library lainnya dapat dipasang melalui Library Manager Arduino IDE.

## Konfigurasi Pin

### LCD dan Keypad

| Fungsi | Pin |
|---|---:|
| I2C SDA | 21 |
| I2C SCL | 47 |
| Keypad baris 1 | 8 |
| Keypad baris 2 | 18 |
| Keypad baris 3 | 17 |
| Keypad baris 4 | 16 |
| Keypad kolom 1 | 15 |
| Keypad kolom 2 | 7 |
| Keypad kolom 3 | 6 |

### Motor dan Encoder

| Fungsi | Pin |
|---|---:|
| Motor kiri maju | 10 |
| Motor kiri mundur | 11 |
| PWM motor kiri | 9 |
| Motor kanan maju | 13 |
| Motor kanan mundur | 14 |
| PWM motor kanan | 12 |
| Encoder kiri | 35 |
| Encoder kanan | 36 |

### Gripper dan Lift

| Fungsi | Pin |
|---|---:|
| Servo gripper | 4 |
| Lift naik | 38 |
| Lift turun | 39 |
| PWM lift | 37 |

## Parameter Robot

Parameter utama yang digunakan dalam program:

| Parameter | Nilai | Keterangan |
|---|---:|---|
| Diameter roda | 0.065 m | Diameter roda robot |
| Jarak roda | 0.142 m | Jarak antara roda kiri dan kanan |
| Encoder per putaran motor | 25 tick | Resolusi encoder |
| Kecepatan linear maksimum | 0.25 m/s | Batas kecepatan maju |
| Kecepatan angular maksimum | 3.5 rad/s | Batas kecepatan rotasi |
| PWM maksimum | 255 | Batas PWM motor |

Nilai-nilai ini dapat disesuaikan dengan dimensi dan karakteristik robot yang digunakan.

## Cara Kerja Program

1. ESP32 melakukan inisialisasi LCD, keypad, motor, encoder, servo, dan lift.
2. Pengguna memilih mode gerak melalui keypad:
   - Tekan `1` untuk mode kurva.
   - Tekan `2` untuk mode rotasi lalu maju.
3. Pengguna memasukkan koordinat target X dan Y dalam sentimeter.
4. Program mengubah input koordinat menjadi meter.
5. Odometry di-reset dari posisi awal `(0, 0)`.
6. Robot bergerak menuju target menggunakan forward dan inverse kinematics.
7. Setelah sampai target, robot menjepit dan mengangkat barang.
8. Robot kembali ke posisi awal.
9. Robot menurunkan dan melepas barang.
10. Program kembali menunggu pemilihan mode berikutnya.

## Cara Input Koordinat

- Masukkan angka melalui keypad.
- Tekan `#` untuk mengonfirmasi input.
- Tekan `*` untuk menghapus input yang sedang diketik.
- Input dianggap dalam satuan sentimeter dan otomatis dikonversi ke meter oleh program.

Contoh:

- Input `50` berarti `50 cm` atau `0.5 m`.
- Input `120` berarti `120 cm` atau `1.2 m`.

## Mode Gerak

### Mode 1: Gerak Kurva

Pada mode ini robot langsung menuju target dengan menghitung jarak dan sudut target secara terus-menerus. Robot menggunakan kontrol proporsional untuk menentukan kecepatan linear dan angular, kemudian inverse kinematics digunakan untuk menghasilkan target kecepatan roda kiri dan kanan.

Mode ini cocok untuk gerakan langsung ke target dengan koreksi lintasan selama robot berjalan.

### Mode 2: Rotasi + Maju

Pada mode ini robot menggunakan state machine dua fase:

1. **Fase rotasi:** robot berputar di tempat sampai menghadap target.
2. **Fase maju:** robot bergerak maju menuju target dengan koreksi sudut kecil.

Mode ini cocok ketika robot perlu menghadap target terlebih dahulu sebelum bergerak lurus.

## Struktur File

```text
.
├── program.ino
└── README.md
```

## Cara Upload ke ESP32

1. Buka Arduino IDE.
2. Buka file `program.ino`.
3. Pilih board ESP32 yang sesuai.
4. Pilih port serial ESP32.
5. Pastikan semua library sudah terpasang.
6. Klik **Upload**.
7. Setelah upload selesai, gunakan keypad dan LCD untuk menjalankan robot.

## Catatan Kalibrasi

Beberapa nilai mungkin perlu dikalibrasi ulang sesuai kondisi robot sebenarnya:

- `DIAMETER_RODA`
- `JARAK_RODA`
- `ENCODER_PER_MOTOR_REV`
- `Kp_linear`
- `Kp_angular`
- `Kp_wheel`
- `ffGainL`
- `ffGainR`
- nilai PWM minimum motor
- durasi rotasi pada fungsi balik home sederhana
- sudut servo saat menjepit dan melepas barang
- durasi lift naik dan turun

Kalibrasi diperlukan karena setiap motor, encoder, mekanik roda, dan permukaan lantai dapat menghasilkan respons yang berbeda.

## Lisensi

Proyek ini dibuat untuk kebutuhan pembelajaran dan praktikum robotika lanjut.
