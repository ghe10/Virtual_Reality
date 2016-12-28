//Joe Snider
//9/12
//
//New version that uses the much improved boost library to handle timing
//
//Warning: the old version could give bad timing information in win7.
//
//Joe Snider 3/12 - Gah ... boost is only giving 1ms resolution on some machines.
//  back to queryperformance (but check it)

#include <windows.h>

#ifndef CPrecisionClockH
#define CPrecisionClockH

using namespace std;

class cPrecisionClock {
public:

	// Constructor of cPrecisionClock.
	cPrecisionClock() {
		QueryPerformanceFrequency(&m_li);
		m_invFreq = 1.0/double(m_li.QuadPart);
		//get the current time
		QueryPerformanceCounter(&m_zeroTime);
	}

	//! Destructor of cPrecisionClock.
	~cPrecisionClock() {}

	//compatibility with the old robot code
	double getCPUTimeSeconds() {
		QueryPerformanceCounter(&m_li);
		return double(m_li.QuadPart - m_zeroTime.QuadPart)*m_invFreq;
	}

private:
	double m_invFreq;
	LARGE_INTEGER m_li;
	LARGE_INTEGER m_zeroTime;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
