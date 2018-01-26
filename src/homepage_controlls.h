#ifndef HOMEPAGE_CONTROLLS_H
#define HOMEPAGE_CONTROLLS_H

#include <QWidget>
#include <QString>

namespace Ui {
class HomepageControlls;
}

namespace adiscope {
class HomepageControlls : public QWidget
{
	Q_OBJECT

public:
	explicit HomepageControlls(QWidget *parent = 0);
	~HomepageControlls();

	void updatePosition();
	virtual bool eventFilter(QObject *, QEvent *);

Q_SIGNALS:
	void goLeft();
	void goRight();
	void openFile();

private:
	Ui::HomepageControlls *ui;
};
}
#endif // HOMEPAGE_CONTROLLS_H
