class Motor
{
public:
	Motor(int npPin, int nnPin, int velPin, bool invert);

	void moveForward();
	void moveForward(unsigned int speed);

	void moveBackward();
	void moveBackward(unsigned int speed);

	void stopMovement();

private:
	int npPin;
	int nnPin;
	int velPin;
	bool invert;
};