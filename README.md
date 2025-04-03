# maze_tracker
 Intelligent Pathfinding System: A multi-tech project integrating pathfinding algorithms, computer vision, and Arduino to navigate real-world road networks. 🗺️ Uses Google Maps screenshots, OpenCV, and AI to solve mazes and control a bot. 🤖🚦

## 📌 Project Overview  

This project is an advanced pathfinding system that integrates multiple technologies to navigate mazes and road networks.  
- **Algorithms for pathfinding** (BFS, DFS, Dijkstra, A*).  
- **Computer Vision (OpenCV) for real-world map processing.**  
- **Arduino integration for bot navigation and traffic signal control.**  

---

## 📂 Project Structure  

This section describes the role of each script in the project.  

### **1️⃣ `algorithms.py` - Pathfinding Algorithms**  
This file contains implementations of fundamental pathfinding algorithms:  
- **Breadth-First Search (BFS)** - Explores all possible routes and finds the shortest path in unweighted maps.  
- **Depth-First Search (DFS)** - Explores paths deeply before backtracking; not optimal for shortest paths.  
- **Dijkstra’s Algorithm** - Finds the shortest path in weighted graphs (road networks).  
- **A* (A-star) Algorithm** - Uses heuristics to improve efficiency in complex environments.  

📌 *This is the core logic for solving mazes, whether they are simulated or real-world maps.*  

---

### **2️⃣ `img_preprocess.py` - Google Maps to Maze Conversion**  
This script takes a **screenshot from Google Maps** and converts it into a **pixel-based maze** using OpenCV.  

📌 **Key Features:**  
- Uses **edge detection** to identify roads.  
- Converts **real-world paths into a format usable by `path_finder.py`.**  

🛠 *This is crucial for applying pathfinding to real-world maps!*  

---

### **3️⃣ `path_finder.py` - Finding the Best Route**  
This is the main script responsible for **executing pathfinding algorithms** from `algorithms.py`.  

📌 **How it Works:**  
- Takes an **image-based maze** from `img_preprocess.py`.  
- Applies **BFS, DFS, Dijkstra, or A*** to compute the best path.  
- Outputs the **optimal route as an overlay on the original map.**  

🚀 *This is the brain of the project, responsible for computing the route!*  

---

### **4️⃣ `road_signals.py` - Computer Vision for Traffic Signals**  
This script processes **real-world road signals** using **computer vision techniques.**  

📌 **How it Works:**  
- Reads **images of road signs** and converts them into **templates.**  
- Uses OpenCV to **recognize traffic signals** and integrate them into the pathfinding system.  
- Helps the bot make decisions based on **traffic rules.**  

🔎 *This allows the bot to interpret road signs like "STOP" or "GO".*  

---

### **5️⃣ `arduino_road_signal.py` - Interface Between Python & Arduino**  
This script acts as a **bridge between Python and Arduino**, allowing communication via serial connection.  

📌 **Key Features:**  
- Sends **pathfinding outputs** to an **Arduino-based road signal system**.  
- Controls **LED signals and displays instructions** for a moving bot.  

🔗 *This is crucial for integrating software decisions with real-world hardware actions.*  

---

### **6️⃣ `bot.ino` - Arduino Bot Code**  
This is the **Arduino firmware** responsible for controlling the **physical bot.**  

📌 **What it Does:**  
- Receives **commands from Python** via serial communication.  
- Moves the bot **forward, backward, left, or right** based on computed paths.  
- Uses **sensors (IR, ultrasonic) for obstacle detection.**  

🤖 *This brings the project into the physical world, making the bot navigate based on computed routes.*  

---

## 🚀 How It Works  

### **Step 1: Convert a Google Maps Screenshot into a Maze**  
```bash
python img_preprocess.py --input maps/screenshot.png --output maps/maze.png
