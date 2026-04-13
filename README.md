# MapiCog
Designed and validated a wearable device that helps visually impaired users build on-demand cognitive maps of their surroundings — without occupying the auditory channel.

🔧 How it works:

• Ultrasonic piezoelectric transducers mounted on a cap capture spatial data (distance, position, and material of nearby objects)

• An ESP32 microcontroller processes the information in real time

• Spatial feedback is transmitted as vibration bursts through bone conduction headphones

• An IMU tracks head movement to enrich spatial awareness

💡 Key design decision — on-demand interaction:
Users request environmental information only when needed, reducing sensory overload while keeping the auditory channel completely free.

📊 Results:

✅ Accuracy rates between 60% and 86% depending on task complexity

✅ Validated with both sighted and visually impaired volunteers

✅ Full prototype built for under $150 USD

🛠️ Built with: ESP32 · Ultrasonic piezoelectric transducers · Bone conduction headphones · Custom PCB · 3D-printed enclosures · IMU · Firebase
