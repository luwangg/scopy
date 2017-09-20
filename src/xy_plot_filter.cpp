#include "xy_plot_filter.h"

using namespace adiscope;

XYPlotFilter::XYPlotFilter():
	cross_lvl(0),
	hyst_span(0),
	cross_detect(nullptr)
{
}

XYPlotFilter::~XYPlotFilter()
{
	delete cross_detect;
}

void XYPlotFilter::filter(double *realDataPoints,
			  double *imagDataPoints,
			  int64_t numDataPoints)
{

	cross_detect = new CrossingDetection(0, 0, "P");

	cross_detect->setHysteresisSpan(hyst_span);
	cross_detect->setLevel(cross_lvl);

	for (size_t i = 1; i < numDataPoints; i++)
		cross_detect->crossDetectStep(realDataPoints, i);

	QList<CrossPoint> periodPoints = cross_detect->detectedCrossings();

	if (periodPoints.size() < 3){
		qDebug() << "We need atleast 3 period points!";
		return;
	}

	int nrOfPeriods = (periodPoints.size() - 1) / 2;
	int firstPoint = periodPoints.first().m_bufIdx;
	int lastPoint = periodPoints.last().m_bufIdx;
	int sampsPerPeriod = (lastPoint - firstPoint + 1) / nrOfPeriods;

	if ((firstPoint + (numDataPoints - lastPoint) + (nrOfPeriods * sampsPerPeriod)) != numDataPoints){
		lastPoint = periodPoints.at(periodPoints.size() - 2).m_bufIdx;
		sampsPerPeriod = (lastPoint - firstPoint) / nrOfPeriods;
		if ((firstPoint + (numDataPoints - lastPoint) + (nrOfPeriods * sampsPerPeriod)) != numDataPoints){
			lastPoint = periodPoints.last().m_bufIdx;
			firstPoint = periodPoints.at(1).m_bufIdx;
			sampsPerPeriod = (lastPoint - firstPoint) / nrOfPeriods;
		}
	}

	if (sampsPerPeriod < 250){
		qDebug() << "The filter needs atleast 200 samples per period to work properly!";
		return;
	}

	for (int i = 1; i < firstPoint; ++i){
		realDataPoints[i] = realDataPoints[i] / nrOfPeriods + realDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
		imagDataPoints[i] = imagDataPoints[i] / nrOfPeriods + imagDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
	}
	for (int k = 0; k < nrOfPeriods; ++k)
		for (int i = k * sampsPerPeriod + firstPoint;
		     i < (k + 1) * sampsPerPeriod + firstPoint; ++i){
			realDataPoints[i] = realDataPoints[i] / nrOfPeriods + realDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
			imagDataPoints[i] = imagDataPoints[i] / nrOfPeriods + imagDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
		}
	for (int i = lastPoint; i < numDataPoints; ++i){
		realDataPoints[i] = realDataPoints[i] / nrOfPeriods + realDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
		imagDataPoints[i] = imagDataPoints[i] / nrOfPeriods + imagDataPoints[i - 1] * (nrOfPeriods - 1) / nrOfPeriods;
	}

}

void XYPlotFilter::prepareFilter(double cross_lvl, double hyst_span)
{
	this->cross_lvl = cross_lvl;
	this->hyst_span = hyst_span;
}
