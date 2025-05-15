// --- Toggle control variables ---
bool closeActive = false;              // Indica si la pinza está cerrando activamente
bool lastCloseButtonState = HIGH;      // Estado previo del botón 1

// ✅ NUEVO: Parámetros PID y variables
double pidInput, pidOutput, pidSetpoint;
double Kp = 0.8, Ki = 0.3, Kd = 0.05;  // Valores iniciales (ajustar según necesidad)
PID gripPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT); // ✅ NUEVO: Objeto PID

// ✅ NUEVO: Configuraciones de fuerza
#define MAX_SAFE_FORCE 1.5  // Fuerza máxima permitida (ajustar según sensores)
#define TARGET_FORCE 1.2     // Fuerza objetivo inicial 

// Configuración pines SPI para sensor TLE5012
#define PIN_SPI1_SS0 94
#define PIN_SPI1_MOSI 69
#define PIN_SPI1_MISO 95
#define PIN_SPI1_SCK 68

tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// Configuración motor BLDC
BLDCMotor motor = BLDCMotor(7, 0.24, 360, 0.000133);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 6, 5, 3);

float target_voltage = -1;

#if ENABLE_MAGNETIC_SENSOR
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
const int CALIBRATION_SAMPLES = 20;
double xOffset = 0, yOffset = 0, zOffset = 0;
#endif

#if ENABLE_COMMANDER
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_voltage, cmd); }
#endif

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // ✅ NUEVO: Configuración PID
  gripPID.SetMode(AUTOMATIC);
  gripPID.SetOutputLimits(-6, 6);  // Coherente con voltage_limit del driver
  gripPID.SetSampleTime(10);       // Actualización cada 10ms

  tle5012Sensor.init();
  motor.linkSensor(&tle5012Sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();
  Serial.println(F("Motor ready."));

#if ENABLE_MAGNETIC_SENSOR
  dut.begin();
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
#endif

  Serial.print("setup done.\n");
#if ENABLE_COMMANDER
  command.add('T', doTarget, "target voltage");
  Serial.println(F("Set the target voltage using serial terminal:"));
#endif
  _delay(1000);
}

void loop() {
#if ENABLE_MAGNETIC_SENSOR
  // Lógica de botones (modificada para PID)
  bool closePressed = (digitalRead(BUTTON1) == LOW);
  if (closePressed && lastCloseButtonState == HIGH) {
    closeActive = !closeActive;
    if(closeActive) pidSetpoint = TARGET_FORCE; // ✅ NUEVO: Resetear setpoint al activar
  }
  lastCloseButtonState = closePressed;

  // Lectura sensores magnéticos
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset;
  y -= yOffset;
  z -= zOffset;
  double fuerzaActual
  fuerzaAnterior=fuerzaActual
  // ✅ NUEVO: Cálculo de fuerza
  double fuerzaActual = sqrt(x*x + y*y + z*z);
  
  // Control PID solo durante el cierre activo
  if (closeActive) {
    pidInput = fuerzaActual;
    gripPID.Compute();  // ✅ NUEVO: Ejecuta cálculo PID
    
    target_voltage = -pidOutput; // Invertido para dirección de cierre
    
    // ✅ NUEVO: Seguridad por sobrefuerza
    if(fuerzaActual > MAX_SAFE_FORCE) {
      target_voltage = 0;
      closeActive = false;
      Serial.println("!OVERFORCE!");
    }
  } else if (digitalRead(BUTTON2) == LOW) {
    target_voltage = 2;    // Apertura manual
  } else {
    target_voltage = 0;    // Detención
  }

  // ✅ NUEVO: Datos para monitorización
  Serial.print(fuerzaActual);
  Serial.print(",");
  Serial.print(pidSetpoint);
  Serial.print(",");
  Serial.print(target_voltage);
  Serial.println("");

#endif

  // Actualización continua del motor
  tle5012Sensor.update();
  motor.loopFOC();
  motor.move(target_voltage);

#if ENABLE_COMMANDER
  command.run();
#endif
}

#if ENABLE_MAGNETIC_SENSOR
void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double temp, valX, valY, valZ;
    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX;
    sumY += valY;
    sumZ += valZ;
    delay(10);
  }
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}
#endif
