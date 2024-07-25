#include <OneWire.h>
#include <DallasTemperature.h>

// Pin assignments
const int heaterPin = 9; // PWM pin for the heater
const int pumpPin = 10; // PWM pin for the pump
const int tempSensorPin = 2;
const int pressureSensorPin = A0;

// OneWire and DallasTemperature objects
OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

// Constants
const float surfaceArea = 10.0; // cm^2
const float latentHeatEthanol = 846; // J/g (example value)
const float latentHeatMethanol = 1100; // J/g (example value)

// Function prototypes
float getEthanolEvapTemp(float pressure);
float getMethanolEvapTemp(float pressure);
float calculateEvaporationRate(float temp, float pressure, float latentHeat);
float linearInterpolate(float x, float x0, float y0, float x1, float y1);

void setup() {
  Serial.begin(9600);
  sensors.begin();
  
  pinMode(heaterPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  
  analogWrite(heaterPin, 0); // Initially turn off heater
  analogWrite(pumpPin, 0); // Initially turn off pump
}

void loop() {
  sensors.requestTemperatures();
  
  // Read temperatures
  float tempFluid = sensors.getTempCByIndex(0);
  float tempEvaporator = sensors.getTempCByIndex(1);
  float tempEvaporate = sensors.getTempCByIndex(2);
  
  // Read pressure
  float pressure = analogRead(pressureSensorPin) * (5.0 / 1023.0); // Convert to pressure units if necessary

  // Display readings
  Serial.print("Fluid Temp: ");
  Serial.println(tempFluid);
  Serial.print("Evaporator Temp: ");
  Serial.println(tempEvaporator);
  Serial.print("Evaporate Temp: ");
  Serial.println(tempEvaporate);
  Serial.print("Pressure: ");
  Serial.println(pressure);

  // Get evaporation temperatures based on current pressure
  float ethanolEvapTemp = getEthanolEvapTemp(pressure);
  float methanolEvapTemp = getMethanolEvapTemp(pressure);

  // Calculate evaporation rates
  float ethanolEvaporationRate = calculateEvaporationRate(tempEvaporator, pressure, latentHeatEthanol);
  float methanolEvaporationRate = calculateEvaporationRate(tempEvaporator, pressure, latentHeatMethanol);
  
  // Display evaporation rates
  Serial.print("Ethanol Evaporation Rate: ");
  Serial.println(ethanolEvaporationRate);
  Serial.print("Methanol Evaporation Rate: ");
  Serial.println(methanolEvaporationRate);

  // Control logic based on the calculated evaporation rates and temperature thresholds
  float heaterPower = 0;
  if (tempEvaporator > ethanolEvapTemp + 2) {
    heaterPower = 0; // Turn off heater
  } else if (tempEvaporator < ethanolEvapTemp - 2) {
    heaterPower = 255; // Full power to heater
  } else {
    heaterPower = map(tempEvaporator, ethanolEvapTemp - 2, ethanolEvapTemp + 2, 255, 0); // Proportional control
  }
  analogWrite(heaterPin, heaterPower);

  // Pump control based on pressure and fluid temperature
  float pumpPower = 0;
  if (pressure > 1.0 || tempFluid > ethanolEvapTemp) {
    pumpPower = 255; // Full power to pump
  } else {
    pumpPower = map(tempFluid, ethanolEvapTemp - 5, ethanolEvapTemp, 0, 255); // Proportional control
  }
  analogWrite(pumpPin, pumpPower);

  // Small delay before next loop iteration
  delay(1000);
}

// Function to get ethanol evaporation temperature based on pressure
float getEthanolEvapTemp(float pressure) {
  // Data points for ethanol evaporation temperature at various pressures
  const float pressureTable[] = {0.5, 1.0, 1.5, 2.0};
  const float tempTable[] = {60.3, 78.0, 92.4, 105.0};
  
  // Find the appropriate interval and interpolate
  for (int i = 0; i < 3; i++) {
    if (pressure >= pressureTable[i] && pressure <= pressureTable[i + 1]) {
      return linearInterpolate(pressure, pressureTable[i], tempTable[i], pressureTable[i + 1], tempTable[i + 1]);
    }
  }
  
  // Return the closest value if out of bounds
  if (pressure < pressureTable[0]) {
    return tempTable[0];
  } else {
    return tempTable[3];
  }
}

// Function to get methanol evaporation temperature based on pressure
float getMethanolEvapTemp(float pressure) {
  // Data points for methanol evaporation temperature at various pressures
  const float pressureTable[] = {0.5, 1.0, 1.5, 2.0};
  const float tempTable[] = {40.5, 64.7, 83.0, 98.0};
  
  // Find the appropriate interval and interpolate
  for (int i = 0; i < 3; i++) {
    if (pressure >= pressureTable[i] && pressure <= pressureTable[i + 1]) {
      return linearInterpolate(pressure, pressureTable[i], tempTable[i], pressureTable[i + 1], tempTable[i + 1]);
    }
  }
  
  // Return the closest value if out of bounds
  if (pressure < pressureTable[0]) {
    return tempTable[0];
  } else {
    return tempTable[3];
  }
}

// Calculate evaporation rate using simplified model
float calculateEvaporationRate(float temp, float pressure, float latentHeat) {
  // Simplified Fredholm integral approach to calculate evaporation rate
  // Evaporation rate E = A * h * (Ts - Ta) / L
  // A is the surface area, h is the heat transfer coefficient, Ts is the surface temperature, Ta is the ambient temperature, L is the latent heat
  
  const float heatTransferCoefficient = 10.0; // W/m^2K (example value)
  const float ambientTemp = 25.0; // Ambient temperature in °C (example value)
  
  // Convert surface area from cm^2 to m^2
  float surfaceAreaM2 = surfaceArea * 1e-4;
  
  // Calculate the heat input
  float heatInput = surfaceAreaM2 * heatTransferCoefficient * (temp - ambientTemp);
  
  // Calculate the evaporation rate
  float evaporationRate = heatInput / latentHeat; // g/s
  
  return evaporationRate;
}

// Linear interpolation function
float linearInterpolate(float x, float x0, float y0, float x1, float y1) {
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}
