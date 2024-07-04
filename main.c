/*
 * Robot_balance2.c
 *
 * Created: 13/06/2024 14:30:37
 * Author : V14-ADA
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <I2C.h>
#include <MPU6050.h>
#include <PID.h>

// Konfigurasi motor driver
#define ENA 1  // Pin OC1A untuk enable motor A
#define ENB 2  // Pin OC1B untuk enable motor B
#define IN1 5  // Pin PC5 untuk pin 1 motor A
#define IN2 4  // Pin PC4 untuk pin 2 motor A
#define IN3 3  // Pin PC3 untuk pin 3 motor B
#define IN4 2  // Pin PC2 untuk pin 4 motor B

// Konfigurasi sensor MPU6050
#define SCL 5  // Pin SCL untuk clock
#define SDA 4  // Pin SDA untuk data

// Konfigurasi PID
#define KP 45  // Konstanta proporsional
#define KI 10  // Konstanta integral
#define KD 2  // Konstanta derivative
#define SETPOINT 0  // Nilai setpoint untuk PID

// Variabel untuk PID
float pidOutput = 0;
float pidInput = 0;
float pidSetpoint = 0;
float pidError = 0;
float pidLastError = 0;

// Variabel untuk sensor MPU6050
MPU6050 mpu6050;

// Variabel untuk motor driver
volatile uint8_t motorState = 0;

// Fungsi untuk mengatur motor
void setMotorState(uint8_t state) {
	if (state == 0) {
		// Motor berhenti
		TCCR1A = 0;
		TCCR1B = 0;
		} else {
		// Motor berjalan
		TCCR1A = (1 << COM1A1) | (1 << COM1B1);
		TCCR1B = (1 << WGM12) | (1 << CS11);
	}
}

// Fungsi untuk mengatur PID
void pidControl() {
	pidInput = mpu6050.getAngle();
	pidError = pidSetpoint - pidInput;
	pidLastError = pidError;
	pidOutput = KP * pidError + KI * pidLastError + KD * (pidError - pidLastError);
	setMotorState(pidOutput > 0 ? 1 : 0);
}

// Fungsi untuk mengatur motor driver
ISR(TIMER1_COMPA_vect) {
	if (motorState == 1) {
		// Motor berjalan
		if (pidOutput > 0) {
			// Motor A berjalan
			PORTC |= (1 << IN1);
			PORTC &= ~(1 << IN2);
			PORTC |= (1 << IN3);
			PORTC &= ~(1 << IN4);
			} else {
			// Motor B berjalan
			PORTC |= (1 << IN1);
			PORTC &= ~(1 << IN2);
			PORTC |= (1 << IN3);
			PORTC &= ~(1 << IN4);
		}
	}
}

int main() {
	// Inisialisasi motor driver
	DDRC |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
	PORTC &= ~(1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

	// Inisialisasi sensor MPU6050
	mpu6050.begin();
	mpu6050.setAccelRange(MPU6050_RANGE_2G);
	mpu6050.setGyroRange(MPU6050_RANGE_250DPS);
	mpu6050.setFilterBandwidth(MPU6050_BAND_250HZ);

	// Inisialisasi PID
	pidSetpoint = SETPOINT;
	pidOutput = 0;
	pidError = 0;
	pidLastError = 0;

	// Inisialisasi timer 1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1);
	TCCR1B = (1 << WGM12) | (1 << CS11);
	OCR1A = 62500;  // 1 kHz

	// Inisialisasi interrupt timer 1
	TIMSK1 |= (1 << OCIE1A);

	// Inisialisasi motor state
	motorState = 0;

	// Loop utama
	while (1) {
		// Baca data sensor MPU6050
		mpu6050.update();

		// Proses PID
		pidControl();

		// Tunggu 1 ms
		delay(1);
	}

	return 0;
}
