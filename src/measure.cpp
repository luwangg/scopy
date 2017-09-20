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

#include "measure.h"
#include "adc_sample_conv.hpp"
#include <qmath.h>
#include <QDebug>

using namespace adiscope;

Measure::Measure(int channel, double *buffer, size_t length):
	m_channel(channel),
	m_buffer(buffer),
	m_buf_length(length),
	m_sample_rate(1.0),
	m_adc_bit_count(0),
	m_cross_level(0),
	m_hysteresis_span(0)
{

	// Create a set of measurements
	m_measurements.push_back(std::make_shared<MeasurementData>("Period",
		MeasurementData::HORIZONTAL, "s",  channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Frequency",
		MeasurementData::HORIZONTAL, "Hz", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Min",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Max",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Peak-peak",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Mean",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Cycle Mean",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("RMS",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Cycle RMS",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("AC RMS",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Area",
		MeasurementData::VERTICAL, "Vs", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Cycle Area",
		MeasurementData::VERTICAL, "Vs", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Low",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("High",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Amplitude",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Middle",
		MeasurementData::VERTICAL, "V", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("+Over",
		MeasurementData::VERTICAL, "%", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("-Over",
		MeasurementData::VERTICAL, "%", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Rise",
		MeasurementData::HORIZONTAL, "s", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("Fall",
		MeasurementData::HORIZONTAL, "s", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("+Width",
		MeasurementData::HORIZONTAL, "s", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("-Width",
		MeasurementData::HORIZONTAL, "s", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("+Duty",
		MeasurementData::HORIZONTAL, "%", channel));
	m_measurements.push_back(std::make_shared<MeasurementData>("-Duty",
		MeasurementData::HORIZONTAL, "%", channel));

}

bool Measure::highLowFromHistogram(double &low, double &high,
		double min, double max)
{
	bool success = false;
	int *hist = m_histogram;
	int adc_span = 1 << m_adc_bit_count;
	int hlf_scale = adc_span / 2;

	int minRaw = adc_sample_conv::convVoltsToSample(min) + hlf_scale;
	int maxRaw = adc_sample_conv::convVoltsToSample(max) + hlf_scale;
	int middleRaw = minRaw + (maxRaw - minRaw)  / 2;

	auto lowIt = std::max_element(hist + minRaw, hist + middleRaw + 1);
	int lowRaw = std::distance(hist, lowIt);

	auto highIt = std::max_element(hist + middleRaw, hist + maxRaw + 1);
	int highRaw = std::distance(hist, highIt);

	/* Use histogram results if High and Low settling levels can be
	   clearly identified (weight of a level should be 5 times
	   greater than a peak weight - there probably is a better method) */

	if (hist[lowRaw] / 5.0 >= hist[minRaw] &&
		hist[highRaw] / 5.0 >= hist[maxRaw]) {
		low = adc_sample_conv::convSampleToVolts(lowRaw - hlf_scale);
		high = adc_sample_conv::convSampleToVolts(highRaw - hlf_scale);
		success = true;
	}

	return success;
}

void Measure::clearMeasurements()
{
	 for (int i = 0; i < m_measurements.size(); i++)
		m_measurements[i]->setMeasured(false);
}

void Measure::setDataSource(double *buffer, size_t length)
{
	m_buffer = buffer;
	m_buf_length = length;
}

void Measure::measure()
{
	clearMeasurements();

	if (!m_buffer || m_buf_length == 0)
		return;

	double period;
	double frequency;
	double rise_time;
	double fall_time;
	double width_p;
	double width_n;
	double duty_p;
	double duty_n;
	double peak_to_peak;
	double min;
	double max;
	double amplitude;
	double high;
	double middle;
	double low;
	double overshoot_p;
	double overshoot_n;
	double mean;
	double cycle_mean;
	double rms;
	double cycle_rms;
	double rms_ac;
	double cycle_rms_ac;
	double area;
	double cycle_area;
	double sum;
	double sqr_sum;

	// Cache buffer address, length, ADC bit count
	double *data = m_buffer;
	size_t data_length = m_buf_length;
	int adc_span = 1 << m_adc_bit_count;
	int hlf_scale = adc_span / 2;
	bool using_histogram_method = (adc_span > 1);

	max = data[0];
	min = data[0];
	sum = data[0];
	sqr_sum = data[0] * data[0];
	m_cross_detect = new CrossingDetection(m_cross_level, m_hysteresis_span,
			"P");
	if (using_histogram_method)
		m_histogram = new int[adc_span]{};

	for (size_t i = 1; i < data_length; i++) {

		// Find level crossings (period detection)
		m_cross_detect->crossDetectStep(data, i);

		// Min
		if (data[i] < min)
			min = data[i];

		// Max
		if (data[i] > max)
			max = data[i];

		// Sum of values
		sum += data[i];

		// Sum of the squares of values
		sqr_sum += data[i] * data[i];

		// Build histogram
		if (using_histogram_method) {
			int raw = hlf_scale + (int)adiscope::adc_sample_conv::
						convVoltsToSample(data[i]);
			if (raw >= 0 && raw  < adc_span)
				m_histogram[raw] += 1;
		}
	}

	m_measurements[MIN]->setValue(min);
	m_measurements[MAX]->setValue(max);

	// Peak-to-Peak
	peak_to_peak = qAbs(max - min);
	m_measurements[PEAK_PEAK]->setValue(peak_to_peak);

	// Mean
	mean = sum / data_length;
	m_measurements[MEAN]->setValue(mean);

	// RMS
	rms = sqrt(sqr_sum / data_length);
	m_measurements[RMS]->setValue(rms);

	// AC RMS
	rms_ac = sqrt((sqr_sum - 2 * mean * sum +
		data_length *  mean * mean) / data_length);
	m_measurements[AC_RMS]->setValue(rms_ac);

	low = min;
	high = max;

	// Try to use Histogram method
	if (using_histogram_method)
		highLowFromHistogram(low, high, min, max);

	// Low, High, Middle, Amplitude, Overshoot positive/negative
	m_measurements[LOW]->setValue(low);
	m_measurements[HIGH]->setValue(high);

	middle = low + (high - low) / 2.0;
	m_measurements[MIDDLE]->setValue(middle);

	amplitude = high - low;
	m_measurements[AMPLITUDE]->setValue(amplitude);

	overshoot_p = (max - high) / amplitude * 100;
	m_measurements[P_OVER]->setValue(overshoot_p);

	overshoot_n = (low - min) / amplitude * 100;
	m_measurements[N_OVER]->setValue(overshoot_n);

	if (m_histogram != NULL) {
		delete[] m_histogram;
		m_histogram = NULL;
	}

	// Find Period / Frequency
	QList<CrossPoint> periodPoints = m_cross_detect->detectedCrossings();
	int n = periodPoints.size();
	if (n > 2) {
		double sample_period;
		double first_hlf_cycl = 0;
		double secnd_hlf_cycl = 0;
		int j = 0;
		int k = 0;

		for (int i = 0; i < n - 1; i++) {
			size_t diff = periodPoints[i + 1].m_bufIdx -
					periodPoints[i].m_bufIdx;
			if (i % 2) {
				secnd_hlf_cycl += diff;
				j++;
				}
			else {
				first_hlf_cycl += diff;
				k++;
				}
		}

		sample_period = first_hlf_cycl / (n / 2) +
				secnd_hlf_cycl / ((n + 1) / 2 - 1);
		period = sample_period * (1 / m_sample_rate);
		m_measurements[PERIOD]->setValue(period);

		frequency = 1 / period;
		m_measurements[FREQUENCY]->setValue(frequency);

		// Find level crossings (10%, 50%, 90%)
		double lowRef = low + (0.1 * amplitude);
		double midRef = low + (0.5 * amplitude);
		double highRef = low + (0.9 * amplitude);

		CrossingDetection cdLow(lowRef, 0.2, "L");
		CrossingDetection cdMid(midRef, 0.2, "M");
		CrossingDetection cdHigh(highRef, 0.2, "H");

		QList<CrossPoint> crossSequence;
		cdLow.setExternalList(&crossSequence);
		cdMid.setExternalList(&crossSequence);
		cdHigh.setExternalList(&crossSequence);

		size_t period_start = periodPoints[0].m_bufIdx;
		size_t period_end = periodPoints[2].m_bufIdx;
		size_t length = period_end - period_start + 1;

		double period_sum = data[period_start];
		double period_sqr_sum = data[period_start] * data[period_start];

		for (size_t i = period_start + 1; i <= period_start + 2 * length; i++) {
			size_t idx = period_start + (i  % length);

			cdLow.crossDetectStep(data, idx);
			cdMid.crossDetectStep(data, idx);
			cdHigh.crossDetectStep(data, idx);
		}

		for (size_t i = period_start + 1; i <= period_end; i++) {
			period_sum += data[i];
			period_sqr_sum += data[i] * data[i];
		}

		for (int i = 1; i < crossSequence.size(); i++) {
			CrossPoint &p0 = crossSequence[i - 1];
			CrossPoint &p1 = crossSequence[i];
			if (p1.m_bufIdx == p1.m_bufIdx && p1.m_onRising == p0.m_onRising) {
				if (p0.m_name == "MR" && p1.m_name == "LR" ||
						p0.m_name == "HR" && p1.m_name == "MR")
					crossSequence.swap(i, i - 1);
				else if (p0.m_name == "MF" && p1.m_name == "HF" ||
						p0.m_name == "LF" && p1.m_name == "MF")
					crossSequence.swap(i, i - 1);
			}
		}

		QString sequence = "";
		for (int i = 0; i < crossSequence.size(); i++)
			sequence += crossSequence[i].m_name;

		QString periodSequence = "LRMRHRHFMFLF";
		int pos = sequence.indexOf(periodSequence);

		if (pos < 0) {
			qDebug() << "Unable to find 2 transitions for each of the 10%, 50%, 90% levels";
		} else {
			pos /= 2;
			CrossPoint &lowRising = crossSequence[pos];
			CrossPoint &midRising = crossSequence[pos + 1];
			CrossPoint &highRising = crossSequence[pos + 2];
			CrossPoint &highFalling = crossSequence[pos + 3];
			CrossPoint &midFalling = crossSequence[pos + 4];
			CrossPoint &lowFalling = crossSequence[pos + 5];

			//Cycle Mean
			cycle_mean = period_sum / length;
			m_measurements[CYCLE_MEAN]->setValue(cycle_mean);

			//Cycle RMS
			cycle_rms = sqrt(period_sqr_sum / length);
			m_measurements[CYCLE_RMS]->setValue(cycle_rms);

			//Area
			area = sum * (1 / m_sample_rate);
			m_measurements[AREA]->setValue(area);

			//Cycle Area
			cycle_area = period_sum * (1 / m_sample_rate);
			m_measurements[CYCLE_AREA]->setValue(cycle_area);

			// Rise Time
			long long rise = (long long)(highRising.m_bufIdx -
					lowRising.m_bufIdx);
			if (rise < 0)
				rise += length;
			rise_time = rise / m_sample_rate;
			m_measurements[RISE]->setValue(rise_time);

			// Fall Time
			long long fall = (long long)(lowFalling.m_bufIdx -
					highFalling.m_bufIdx);
			if (fall < 0)
				fall += length;
			fall_time = fall / m_sample_rate;
			m_measurements[FALL]->setValue(fall_time);

			// Positive Width
			long long posWidth = (long long)(midFalling.m_bufIdx -
					midRising.m_bufIdx);
			if (posWidth < 0)
				posWidth += length;
			width_p = posWidth / m_sample_rate;
			m_measurements[P_WIDTH]->setValue(width_p);

			// Negative Width
			width_n = period - width_p;
			m_measurements[N_WIDTH]->setValue(width_n);

			// Positive Duty
			duty_p = width_p / period * 100;
			m_measurements[P_DUTY]->setValue(duty_p);

			// Negative Duty
			duty_n = width_n / period * 100;
			m_measurements[N_DUTY]->setValue(duty_n);
		}
	}

	delete m_cross_detect;
	m_cross_detect = NULL;

}

double Measure::sampleRate()
{
	return m_sample_rate;
}

void Measure::setSampleRate(double value)
{
	m_sample_rate = value;
}

unsigned int Measure::adcBitCount()
{
	return m_adc_bit_count;
}

void Measure::setAdcBitCount(unsigned int val)
{
	m_adc_bit_count = val;
}

double Measure::crossLevel()
{
	return m_cross_level;
}

void Measure::setCrossLevel(double value)
{
	m_cross_level = value;
}

double Measure::hysteresisSpan()
{
	return m_hysteresis_span;
}

void Measure::setHysteresisSpan(double value)
{
	m_hysteresis_span = value;
}

int Measure::channel() const
{
	return m_channel;
}

void Measure::setChannel(int channel)
{
	if (m_channel != channel) {
		for (int i = 0; i < m_measurements.size(); i++) {
			m_measurements[i]->setChannel(channel);
		}
		m_channel = channel;
	}
}

QList<std::shared_ptr<MeasurementData>> Measure::measurments()
{
	return m_measurements;
}

std::shared_ptr<MeasurementData> Measure::measurement(int id)
{
	return m_measurements[id];
}

int Measure::activeMeasurementsCount() const
{
	int count = 0;

	for (int i = 0; i < m_measurements.size(); i++)
		if (m_measurements[i]->enabled())
			count++;

	return count;
}

/*
 * Class MeasurementData implementation
 */

MeasurementData::MeasurementData(const QString& name, axisType axis,
		const QString& unit, int channel):
	m_name(name),
	m_value(0),
	m_measured(false),
	m_enabled(false),
	m_unit(unit),
	m_unitType(DIMENSIONLESS),
	m_channel(channel),
	m_axis(axis)
{
	if (unit.isEmpty())
		m_unitType = DIMENSIONLESS;
	else if (unit == "%")
		m_unitType = PERCENTAGE;
	else if (unit.toLower() == "s" || unit.toLower() == "seconds")
		m_unitType = TIME;
	else
		m_unitType = METRIC;
}

QString MeasurementData::name() const
{
	return m_name;
}

double MeasurementData::value() const
{
	return m_value;
}

void MeasurementData::setValue(double value)
{
	m_value = value;
	m_measured = true;
}

bool MeasurementData::measured() const
{
	return m_measured;
}

void MeasurementData::setMeasured(bool state)
{
	m_measured = state;
}

bool MeasurementData::enabled() const
{
	return m_enabled;
}

void MeasurementData::setEnabled(bool en)
{
	m_enabled = en;
}

QString MeasurementData::unit() const
{
	return m_unit;
}

MeasurementData::unitTypes MeasurementData::unitType() const
{
	return m_unitType;
}

int MeasurementData::channel() const
{
	return m_channel;
}

void MeasurementData::setChannel(int channel)
{
	m_channel = channel;
}

MeasurementData::axisType MeasurementData::axis() const
{
	return m_axis;
}

/*
 * Class Statistic implementation
 */

Statistic::Statistic():
	m_sum(0),
	m_min(0),
	m_max(0),
	m_dataCount(0),
	m_average(0)
{
}

void Statistic::pushNewData(double data)
{
	m_sum += data;

	if (!m_dataCount) {
		m_min = data;
		m_max = data;
	} else {
		if (data < m_min)
			m_min = data;
		else if (data > m_max)
			m_max = data;
	}

	m_dataCount += 1;
	m_average = m_sum / m_dataCount;
}

void Statistic::clear()
{
	m_sum = 0;
	m_min = 0;
	m_max = 0;
	m_dataCount = 0;
	m_average = 0;
}

double Statistic::average() const
{
	return m_average;
}

double Statistic::min() const
{
	return m_min;
}

double Statistic::max() const
{
	return m_max;
}

double Statistic::numPushedData() const
{
	return m_dataCount;
}
