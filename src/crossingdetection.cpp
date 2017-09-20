#include "crossingdetection.h"

using namespace adiscope;

CrossPoint::CrossPoint(float value, size_t bufIndex, bool onRising, QString name):
	m_value(value),
	m_bufIdx(bufIndex),
	m_onRising(onRising),
	m_name(name)
{
}

HystLevelCross::HystLevelCross():
	m_low_trhold_crossed(false),
	m_high_trhold_crossed(false),
	m_is_between_trholds(false)
{
}


bool HystLevelCross::isBetweenThresholds()
{
	return m_is_between_trholds;
}

HystLevelCross::crossEvents HystLevelCross::get_crossing_type(double samp, double prevSamp,
							      double low_trhold, double high_trhold)
{
	enum crossEvents cross_type = NO_CROSS;

	if (samp > prevSamp) {
		if (prevSamp <= low_trhold && samp >= low_trhold)
			cross_type = POS_CROSS_LOW;
		if (prevSamp <= high_trhold && samp >= high_trhold)
			if (cross_type == POS_CROSS_LOW)
				cross_type = POS_CROSS_FULL;
			else
				cross_type = POS_CROSS_HIGH;
	} else if (samp < prevSamp) {
		if (prevSamp >= low_trhold && samp <= low_trhold)
			cross_type = NEG_CROSS_LOW;
		if (prevSamp >= high_trhold && samp <= high_trhold)
			if (cross_type == NEG_CROSS_LOW)
				cross_type = NEG_CROSS_FULL;
			else
				cross_type = NEG_CROSS_HIGH;
	}

	return cross_type;
}

void HystLevelCross::resetState()
{
	m_low_trhold_crossed = false;
	m_high_trhold_crossed = false;
	m_is_between_trholds = false;
}

HystLevelPosCross::HystLevelPosCross():
	HystLevelCross()
{
}


bool HystLevelPosCross::updateState(HystLevelCross::crossEvents crsEvent)
{
	bool level_crossed = false;

	switch (crsEvent) {
	case POS_CROSS_LOW:
		m_is_between_trholds = true;
		break;
	case POS_CROSS_HIGH:
		if (m_is_between_trholds) {
			level_crossed = true;
			m_is_between_trholds = false;
		}
		break;
	case POS_CROSS_FULL:
		level_crossed = true;
		break;
	case NEG_CROSS_LOW:
		m_is_between_trholds = false;
	default:
		break;
	}

	return level_crossed;
}

HystLevelNegCross::HystLevelNegCross():
	HystLevelCross()
{
}

bool HystLevelNegCross::updateState(HystLevelCross::crossEvents crsEvent)
{
	bool level_crossed = false;

	switch (crsEvent) {
	case NEG_CROSS_HIGH:
		m_is_between_trholds = true;
		break;
	case NEG_CROSS_LOW:
		if (m_is_between_trholds) {
			level_crossed = true;
			m_is_between_trholds = false;
		}
		break;
	case NEG_CROSS_FULL:
		level_crossed = true;
		break;
	case POS_CROSS_HIGH:
		m_is_between_trholds = false;
	default:
		break;
	}

	return level_crossed;
}

CrossingDetection::CrossingDetection(double level, double hysteresis_span,
				     const QString &name):
	m_posCrossFound(false),
	m_negCrossFound(false),
	m_level(level),
	m_hysteresis_span(hysteresis_span),
	m_low_level(level - hysteresis_span / 2),
	m_high_level(level + hysteresis_span / 2),
	m_name(name),
	m_externList(NULL)
{
}

void CrossingDetection::setLevel(double level)
{
	if (m_level != level) {
		m_level = level;
		m_low_level = level - m_hysteresis_span / 2;
		m_high_level = level + m_hysteresis_span / 2;
	}
}

double CrossingDetection::hysteresisSpan()
{
	return m_hysteresis_span;
}

void CrossingDetection::setHysteresisSpan(double span)
{
	if (m_hysteresis_span != span) {
		m_hysteresis_span = span;
		m_low_level = m_level - span / 2;
		m_high_level = m_level + span / 2;
	}
}

void CrossingDetection::setExternalList(QList<CrossPoint> *externList)
{
	m_externList = externList;
}

QList<CrossPoint> CrossingDetection::detectedCrossings()
{
	return m_detectedCrossings;
}

void CrossingDetection::store_closest_val_to_cross_lvl(double *data, size_t i, size_t &point)
{
	double diff1 = qAbs(data[i - 1] - m_level);
	double diff2 = qAbs(data[i] - m_level);
	double diff;
	size_t idx;

	if (diff1 < diff2) {
		idx = i - 1;
		diff = diff1;
	} else {
		idx = i;
		diff = diff2;
	}

	double old_diff = qAbs(data[point] - m_level);
	if (diff < old_diff)
		point = idx;
}

void CrossingDetection::store_first_closest_val_to_cross_lvl(double *data, size_t i, size_t &point)
{
	double diff1 = qAbs(data[i - 1] - m_level);
	double diff2 = qAbs(data[i] - m_level);

	if (diff1 < diff2)
		point = i - 1;
	else
		point = i;
}

void CrossingDetection::crossDetectStep(double *data, size_t i)
{
	auto cross_type = HystLevelCross::get_crossing_type(data[i],
							    data[i - 1], m_low_level, m_high_level);

	if (m_posCross.isBetweenThresholds())
		store_closest_val_to_cross_lvl(data, i, m_posCrossPoint);
	if (m_negCross.isBetweenThresholds())
		store_closest_val_to_cross_lvl(data, i, m_negCrossPoint);

	if (cross_type != HystLevelCross::NO_CROSS) {
		if (!m_posCrossFound) {
			bool old_between_thresh = m_posCross.isBetweenThresholds();
			m_crossed = m_posCross.updateState(cross_type);
			if (!old_between_thresh && m_posCross.isBetweenThresholds())
				store_first_closest_val_to_cross_lvl(data, i, m_posCrossPoint);

			if (m_crossed) {
				m_posCrossFound = true;
				m_negCrossFound = false;
				m_negCross.resetState();
				if (cross_type == HystLevelCross::POS_CROSS_FULL)
					m_posCrossPoint = i;
				m_detectedCrossings.push_back(
							CrossPoint(data[m_posCrossPoint], m_posCrossPoint,
								   true, m_name + "R"));
				if (m_externList)
					m_externList->push_back(m_detectedCrossings.last());
			}
		}
		if (!m_negCrossFound) {
			bool old_between_thresh = m_negCross.isBetweenThresholds();
			m_crossed = m_negCross.updateState(cross_type);
			if (!old_between_thresh && m_negCross.isBetweenThresholds())
				store_first_closest_val_to_cross_lvl(data, i, m_negCrossPoint);
			if (m_crossed) {
				m_negCrossFound = true;
				m_posCrossFound = false;
				m_posCross.resetState();
				if (cross_type == HystLevelCross::NEG_CROSS_FULL)
					m_negCrossPoint = i - 1;
				m_detectedCrossings.push_back(
							CrossPoint(data[m_negCrossPoint], m_negCrossPoint,
								   false, m_name + "F"));
				if (m_externList)
					m_externList->push_back(m_detectedCrossings.last());
			}
		}
	}
}

double CrossingDetection::level()
{
	return m_level;
}

