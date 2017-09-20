#ifndef OSC_PLOT_FILTER_H
#define OSC_PLOT_FILTER_H

#include "crossingdetection.h"
#include <QDebug>

namespace adiscope {
	class XYPlotFilter
	{
	public:
		XYPlotFilter();
		~XYPlotFilter();

		void filter(double *realDataPoints,
				double *imagDataPoints,
				int64_t numDataPoints);

		void prepareFilter(double cross_lvl,
				   double hyst_span);

	private:
		CrossingDetection *cross_detect;
		double cross_lvl;
		double hyst_span;
	};
}

#endif // OSC_PLOT_FILTER_H
