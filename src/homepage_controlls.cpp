#include "homepage_controlls.h"
#include "ui_homepage_controlls.h"
#include <QDebug>

using namespace adiscope;

HomepageControlls::HomepageControlls(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::HomepageControlls)
{
	ui->setupUi(this);

	connect(ui->leftBtn, &QPushButton::clicked, [=](){
		Q_EMIT goLeft();
	});
	connect(ui->rightBtn, &QPushButton::clicked, [=](){
		Q_EMIT goRight();
	});
	connect(ui->openBtn, &QPushButton::clicked, [=](){
		Q_EMIT openFile();
	});

	ui->openBtn->hide();

	updatePosition();
}

HomepageControlls::~HomepageControlls()
{
	delete ui;
}

void HomepageControlls::updatePosition()
{
	move(parentWidget()->width() - 150, geometry().topLeft().y());
}

bool HomepageControlls::eventFilter(QObject *watched, QEvent *event)
{
	if (event->type() == QEvent::Resize) {
		updatePosition();
		return false;
	}

	return QObject::eventFilter(watched, event);
}
