# Node Explanation
Node ini (/node) berfungsi untuk melakuakn subscribe dari tiga topic dengan tiga jenis input Twist yaitu:

1. Keyboard (/keyboard_vel) -> memiliki prioritas 1 (tertinggi)
2. Joystick (/joy_vel) -> prioritas 2
3. Autonomous (/autonomous_vel) -> prioritas 2 (terendah)

Node ini (/node) kemudian adan melakukan publish ke 2 topic yaitu:
1. /cmd_type 
2. /cmd_vel

Node ini menggunakan priority queue dan threading lock agar semua message yang diterima diproses satu per satu sesuai urutan prioritas. Node ini menyimpan message dari berbagai sumber beserta prioritasnya, dan kemudian setiap 0.2 detik akan mengirim dengan prioritas tertinggi ke topik /cmd_vel dan /cmd_type.

## How to run the code
1. Build workspace
    colcon build
    source install/setup.bash
2. Jalankan Nodes dalam package ‘magang_2025’ dengan perintah:
    ros2 launch magang_2025 milestone2.launch.py
3. Jalankan Node dalam package Anda dengan perintah berikut pada terminal baru:
    ros2 launch pkg_13524097 launch.py
4. 
![alt text](diagram.drawio.png.png)