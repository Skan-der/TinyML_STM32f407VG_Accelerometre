# Tilt Detection Using STM32F407VG and TensorFlow Lite

## Overview
This project demonstrates how to integrate a machine learning model with the STM32F407VG microcontroller to detect the tilt of the board. The workflow includes training the model, converting it to TensorFlow Lite format, and implementing it on the STM32 platform. The repository includes all the necessary code, a presentation, and a demo video showcasing the project's functionality.

## Features
- **Machine Learning Model**: A custom-trained model developed using TensorFlow and converted to TensorFlow Lite for embedded applications.  
- **Tilt Detection**: Detects tilt in multiple directions using accelerometer data.  
- **STM32 Integration**: Implementation of the model using STM32CubeIDE and X-Cube-AI.  
- **Real-Time Feedback**: LEDs on the STM32 board indicate tilt direction.  

## Repository Contents
- **`model_training/`**: Contains the Google Colab notebook used to train the model and export it as a `.tflite` file.  
- **`stm32_code/`**: Source code for the STM32F407VG board, implemented using STM32CubeIDE.  
- **`presentation/`**: A detailed presentation explaining the project's workflow and results.  
- **`demo_video/`**: A video demonstration of the tilt detection system in action.  

## Getting Started
1. **Model Training**:  
   Open the `model_training` notebook in Google Colab to understand how the model was trained. Follow the steps to retrain or modify the model if needed.

2. **STM32 Setup**:  
   - Import the STM32CubeIDE project from the `stm32_code` folder.  
   - Ensure the required peripherals (UART, SPI, GPIO, etc.) are configured.  
   - Build and flash the code onto the STM32F407VG board.  

3. **Hardware Requirements**:  
   - STM32F407VG Discovery Board.  
   - LIS3DSH accelerometer (integrated into the board).  

4. **Running the Project**:  
   - Tilt the STM32 board in different directions.  
   - Observe the LEDs indicating the detected tilt direction.  

## Demo
A video demonstration of the project can be found in the `demo_video` folder.  

## How It Works
1. **Data Collection**: Accelerometer data is collected using the LIS3DSH sensor.  
2. **Inference**: The data is fed into the TensorFlow Lite model deployed on the STM32 board.  
3. **Output**: The model predicts the tilt direction, and corresponding LEDs light up.  

## Tools and Technologies
- **TensorFlow** for model training.  
- **TensorFlow Lite** for model optimization.  
- **STM32CubeIDE** and **X-Cube-AI** for firmware development.  

## Future Improvements
- Enhance the model's accuracy by collecting more diverse training data.  
- Optimize the code for lower power consumption.  
- Extend the project to detect more complex movements.  

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
