/*
 * Copyright 2016 Analog Devices, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file LICENSE.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef MEASURE_H
#define MEASURE_H

#include <QList>

namespace adiscope {
	class CrossingDetection;

	class MeasurementData
	{
	public:
		enum unitTypes {
			METRIC,
			TIME,
			PERCENTAGE,
			DIMENSIONLESS
		};

		MeasurementData(const QString& name, const QString& unit = "");

		QString name() const;
		double value() const;
		void setValue(double value);
		bool measured() const;
		void setMeasured(bool state);
		bool enabled() const;
		void setEnabled(bool en);
		QString unit() const;
		enum unitTypes unitType() const;

	private:
		QString m_name;
		double m_value;
		bool m_measured;
		bool m_enabled;
		QString m_unit;
		enum unitTypes m_unitType;
	};

	class Measure
	{
	public:
		enum defaultMeasurements {
			PERIOD = 0,
			FREQUENCY,
			MIN,
			MAX,
			PEAK_PEAK,
			MEAN,
			RMS,
			AC_RMS,
			LOW,
			HIGH,
			AMPLITUDE,
			MIDDLE,
			P_OVER,
			N_OVER,
			RISE,
			FALL,
			P_WIDTH,
			N_WIDTH,
			P_DUTY,
			N_DUTY,
			DEFAULT_MEASUMENET_COUNT
		};

		Measure(double *buffer = NULL, size_t length = 0);

		void setDataSource(double *buffer, size_t length);
		void measure();
		double sampleRate();
		void setSampleRate(double);
		unsigned int adcBitCount();
		void setAdcBitCount(unsigned int);
		double crossLevel();
		void setCrossLevel(double);
		double hysteresisSpan();
		void setHysteresisSpan(double);

		const QList<MeasurementData>& measurements();
		void setMeasurementEnabled(int measure_idx, bool en);

	private:
		bool highLowFromHistogram(double &low, double &high,
			double min, double max);
		void clearMeasurements();

	private:
		double *m_buffer;
		size_t m_buf_length;
		double m_sample_rate;
		unsigned int m_adc_bit_count;
		double m_cross_level;
		double m_hysteresis_span;

		int *m_histogram;
		CrossingDetection *m_cross_detect;

		QList<MeasurementData> m_measurements;
	};
}

#endif // MEASURE_H
