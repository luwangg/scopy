#ifndef CROSSINGDETECTION_H
#define CROSSINGDETECTION_H

#include <QList>
#include <QString>
#include <memory>

namespace adiscope {
	class CrossPoint
	{
		public:
		CrossPoint(float value, size_t bufIndex, bool onRising, QString name);

	public:
		float m_value;
		size_t m_bufIdx;
		bool m_onRising;
		QString m_name;
	};

	class HystLevelCross
	{
	public:
		enum crossEvents {
			NO_CROSS = 0,
			POS_CROSS_LOW,
			POS_CROSS_HIGH,
			POS_CROSS_FULL,
			NEG_CROSS_LOW,
			NEG_CROSS_HIGH,
			NEG_CROSS_FULL,
		};

		HystLevelCross();

		bool isBetweenThresholds();

		virtual inline bool updateState(enum crossEvents crsEvent) = 0;

		static inline enum crossEvents
			get_crossing_type(double samp, double prevSamp,
					double low_trhold, double high_trhold);

		void resetState();

	protected:
		bool m_low_trhold_crossed;
		bool m_high_trhold_crossed;
		bool m_is_between_trholds;
	};

	class HystLevelPosCross: public HystLevelCross
	{
	public:
		HystLevelPosCross();
		inline bool updateState(enum crossEvents crsEvent);
	};

	class HystLevelNegCross: public HystLevelCross
	{
	public:
		HystLevelNegCross();
		inline bool updateState(enum crossEvents crsEvent);
	};

	class CrossingDetection
	{
	public:
		CrossingDetection(double level, double hysteresis_span,
				const QString &name);
		double level();

		void setLevel(double level);

		double hysteresisSpan();

		void setHysteresisSpan(double span);

		void setExternalList(QList<CrossPoint> *externList);

		QList<CrossPoint> detectedCrossings();

		void store_closest_val_to_cross_lvl(double *data, size_t i, size_t &point);

		void store_first_closest_val_to_cross_lvl(double *data, size_t i, size_t &point);

		void crossDetectStep(double *data, size_t i);

	private:
		HystLevelPosCross m_posCross;
		HystLevelNegCross m_negCross;

		bool m_posCrossFound;
		bool m_negCrossFound;
		bool m_crossed;

		double m_level;
		double m_hysteresis_span;
		double m_low_level;
		double m_high_level;

		size_t m_posCrossPoint;
		size_t m_negCrossPoint;

		QList<CrossPoint> m_detectedCrossings;
		QList<CrossPoint> *m_externList;

		QString m_name;
	};
}

#endif // CROSSINGDETECTION_H
