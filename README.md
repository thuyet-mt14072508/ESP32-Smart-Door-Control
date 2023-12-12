# **ESP32 Smart Door Lock**

**Mục lục**

- [Giới thiệu]
- [Phần cứng]
- [Tính năng phần mềm]
  - [Đăng ký]
  - [Xác thực]
  - [Tích hợp MQTT]
- [Cải tiến]
**Giới thiệu**

Đây là dự án thực hiện hệ thống khóa cửa thông minh sử dụng bo mạch phát triển ESP32. Hệ thống hỗ trợ 3 phương thức xác thực gồm mật khẩu qua bàn phím, dấu vân tay và thẻ RFID.

**Phần cứng**

- **Vi điều khiển** : ESP32 DevKitC V4
- **Màn hình LCD** : LCD 20x4 kết nối I2C
  - Hiển thị trạng thái và thông báo
- **Bàn phím** : Bàn phím màn hình 4x4
  - Nhập mật khẩu đăng ký và xác thực
- **Cảm biến vân tay** : GT-521F32 quang học
  - Đăng ký và nhận dạng vân tay
- **Đọc thẻ RFID** : MFRC522
  - Đăng ký và xác nhận thẻ RFID

**Tính năng phần mềm**

Đăng ký

Thiết bị cung cấp menu trên màn LCD để:

- Thiết lập mật khẩu 4-6 ký tự qua bàn phím
- Ghi danh vân tay
- Đăng ký thẻ RFID

Dữ liệu được lưu tạm trong bộ nhớ. Có thể mở rộng lưu vào SPIFFS hoặc EEPROM.

Xác thực

3 task FreeRTOS chạy song song giám sát bàn phím, cảm biến vân tay và đầu đọc RFID.

Khi xác thực thành công sẽ hiển thị "Cho phép truy cập" trên LCD và gửi thông báo lên MQTT. Nếu thất bại hiển thị "Từ chối truy cập".

Tích hợp MQTT

Thiết bị kết nối MQTT qua WiFi để:

- Gửi thông báo mỗi lần xác thực
- Đăng ký mật khẩu, vân tay, thẻ RFID mới

Có thể mở rộng tích hợp Adafruit IO để đẩy lên dashboard.

**Cải tiến**

- Lưu trữ dữ liệu đăng ký vào bộ nhớ không mất điện
- Thêm kết nối BLE thay cho WiFi+MQTT
- Thiết kế vỏ bọc bằng máy in 3D
- Điều khiển servo mô phỏng mở khóa cửa

Hãy góp ý nếu cần bổ sung thông tin gì!
