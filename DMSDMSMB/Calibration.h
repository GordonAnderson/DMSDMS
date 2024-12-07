#ifndef Calibration_h
#define Calibration_h

void CalibrateREF(void);

void CalibrateDCBA1(void);
void CalibrateDCBB1(void);

void CalibrateDCBA2(void);
void CalibrateDCBB2(void);

void CalibrateVrf(int ch);
void CalibrateVrf2Drive(int ch);

void calCurrent(int ch);

#endif
