#ifndef GAMEPADDRIVER_H
#define GAMEPADDRIVER_H
//


class Joystick;

class GamePadDriver : public QThread
{
public:
	GamePadDriver(Session* s);
	~GamePadDriver();
	void run();
private:
	void normValues();
	double normv(int val, int maxv, int minv, int max0v, int min0v);
	bool connectToGamePad();
	QTimer* timer;
	Session* session;
	Joystick* joy;
	string devPort;
	string configFile;
	
	vector<double> normAxisValues;
	vector<int> axisValues;
	QVector<bool> buttonValues;
	
	QVector<int> maxAxisValues;
	QVector<int> minAxisValues;
	QVector<int> maxAxisZeroValues;
	QVector<int> minAxisZeroValues;

	int gamepadMode;
	
private slots:
	void slot_readGamePad();
};
#endif
