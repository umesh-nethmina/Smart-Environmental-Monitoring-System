# 🌍 Smart Solar-Powered Environmental Monitoring System

This project is an **IoT-based environmental monitoring system** using ESP32 + GSM SIM900A + multiple sensors.  
It works completely on **solar power with a Li-ion battery pack**, making it suitable for remote and rural areas.

---

## ⚡ Features
- ✅ Sensors: AHT10 (Temp/Humidity), BMP280 (Pressure), PMS5003 (PM2.5/PM10), MH-Z19C (CO₂), MICS5524 (CO)
- ✅ GSM SIM900A for GPRS internet connection
- ✅ Data uploaded to **ThingSpeak / Google Cloud**
- ✅ DS3231 RTC for accurate timestamps
- ✅ W25Q128 Flash for offline buffering
- ✅ SMS alerts on GSM reconnect
- ✅ Powered by **12W Solar Panel + 2S3P Battery Pack**

---

## 🔋 Power System
- **Battery Pack:** 2S3P (6× 18650, 3200 mAh each → 9600 mAh total, 7.4V nominal, 71 Wh)  
- **Charger:** TP5100 (CC/CV control, safe charging)  
- **Protection:** 2S BMS (overcharge/discharge protection)

---

## 📡 How It Works
1. Sensors collect environmental data every 5 minutes.  
2. ESP32 processes the data and sends via GSM module.  
3. If GPRS available → HTTP GET request uploads data to ThingSpeak.  
4. If GPRS fails → data is stored in Flash with RTC timestamp.  
5. When GSM reconnects → buffered data is automatically uploaded.  
6. System sends SMS with downtime info + last values.

---

## 📊 Applications
- 🌱 Smart Agriculture  
- 🏙️ Smart City Air Quality Monitoring  
- 🏞️ Remote Environmental Stations  
- 🔬 Research & Education Projects  

---

## 📸 Project Demo
<img width="639" height="776" alt="image" src="https://github.com/user-attachments/assets/0ce96295-c7c1-4d30-8f65-6704f1910272" />
<img width="646" height="861" alt="image" src="https://github.com/user-attachments/assets/defddd80-ed62-4ebf-96a6-ea2ba2b625a2" />
<img width="633" height="845" alt="image" src="https://github.com/user-attachments/assets/5ad245bb-9296-4012-853f-071253e8ca57" />

---

## 🛠️ Hardware List
- ESP32  
- SIM900A GSM Module  
- Sensors: AHT10, BMP280, PMS5003, MH-Z19C, MICS5524  
- DS3231 RTC  
- W25Q128 Flash memory  
- 2S3P 18650 battery pack + TP5100 charger + Solar Panel 12W  

---

## 📂 Files Included
- `code/` → Arduino source code  
- `hardware/` → Circuit diagrams & pack design  
- `docs/` → Research paper, Report, Slides  

---

## 👨‍💻 Author
Developed by **[umesh nethmina]**  
📧 [umeshnethmina118@gmail.com]  
🔗 LinkedIn: [linkedin.com/in/umesh-nethmina-b76582306]
