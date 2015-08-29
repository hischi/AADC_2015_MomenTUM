// CodeCogs Commercial License Agreement
// Copyright (C) 2004-2010 CodeCogs, Zyba Ltd, Broadwood, Holford, TA5 1DU, England.
//
// This software is licensed to Markus Fichtner 
// for commercial usage by version 1.2.1 of the CodeCogs Commercial Licence. You must 
// read this License (available at www.codecogs.com) before using this software.
//
// If you distribute this file it is YOUR responsibility to ensure that all 
// recipients have a valid number of commercial licenses. You must retain a
// copy of this licence in all copies you make. 
//
// This program is distributed WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the CodeCogs Commercial Licence for more details.
//---------------------------------------------------------------------------------
//! Interpolates a given set of points using cubic spline fitting.

#ifndef MATHS_INTERPOLATION_CUBIC_H
#define MATHS_INTERPOLATION_CUBIC_H


//! Interpolates a given set of points using cubic spline fitting.

class Cubic{
public:

struct Pair{
    tFloat32 x;
    tFloat32 y;
    bool operator() (Pair i,Pair j) { return (i.x<j.x);}
}pair;

//! Class constructor with pair
Cubic(vector<Pair> val){
    std::sort(val.begin(),val.end(),pair);
    m_rangeStart= val[0].x;
    m_rangeStop=val[val.size()-1].x;
    int n  = val.size();
    m_n = n;
    m_x = new tFloat64[m_n + 1];
    m_y = new tFloat64[m_n + 1];
    m_b = new tFloat64[m_n + 1];
    m_c = new tFloat64[m_n + 1];
    m_d = new tFloat64[m_n + 1];

    // remember and shift base.
    for (tInt i = 1; i <= m_n; ++i) {
            m_x[i] = val[i-1].x;
            m_y[i] = val[i - 1].y;
    }

  // linear interpolation when we have too little data (i.e. just two points!)
    if (m_n < 3) {
            m_b[2] = m_b[1] = (m_y[2] - m_y[1]) / (m_x[2] - m_x[1]);
            m_c[1] = m_c[2] = m_d[1] = m_d[2] = 0.0;
            return;
    }


    m_d[1] = m_x[2] - m_x[1];
    m_b[1] = - m_d[1];
    m_c[2] = (m_y[2] - m_y[1]) / m_d[1];
    m_c[1] = 0.0;
    for (tInt i = 2; i < m_n; ++i) {
      m_d[i] = m_x[i + 1] - m_x[i];
      m_b[i] = 2.0 * (m_d[i - 1] + m_d[i]);
      m_c[i + 1] = (m_y[i + 1] - m_y[i]) / m_d[i];
      m_c[i] = m_c[i + 1] - m_c[i];
    }

    m_b[m_n] = -m_d[m_n - 1];
    m_c[m_n] = 0.0;
    if (m_n != 3) {
        m_c[1] = m_c[3] / (m_x[4] - m_x[2]) - m_c[2] / (m_x[3] - m_x[1]);
        m_c[n] = m_c[n - 1] / (m_x[n] - m_x[n - 2]) - m_c[n - 2] / (m_x[n - 1] - m_x[n - 3]);
        m_c[1] *= m_d[1] * m_d[1] / (m_x[4] - m_x[1]);
        m_c[n] *= - m_d[n - 1] * m_d[n - 1] / (m_x[n] - m_x[n - 3]);
    }
    for (tInt i = 2; i < m_n; ++i) {
        tFloat64 T = m_d[i - 1] / m_b[i - 1];
        m_b[i] -= T * m_d[i - 1];
        m_c[i] -= T * m_c[i - 1];
    }

    m_c[m_n] /= m_b[m_n];
    for  (tInt i = n - 1; i > 0; --i)
            m_c[i] = (m_c[i] - m_d[i] * m_c[i + 1]) / m_b[i];

    m_b[n] = (m_y[n] - m_y[n - 1]) / m_d[n - 1] + m_d[n - 1] * (m_c[n - 1] + 2.0 * m_c[n]);

    for (tInt i = 1; i < n; ++i) {
      m_b[i] = (m_y[i + 1] - m_y[i]) / m_d[i] - m_d[i] * (m_c[i + 1] + 2.0 * m_c[i]);
      m_d[i] = (m_c[i + 1] - m_c[i]) / m_d[i];
      m_c[i] *= 3.0;
    }
    m_c[n] *= 3.0;
    m_d[n] = m_d[n - 1];

}


//! Class constructor

Cubic(tInt n, vector<tFloat32> x, vector<tFloat32> y, tFloat32 rangeStart, tFloat32 rangeStop)
{
    m_rangeStart = rangeStart;
    m_rangeStop = rangeStop;

	m_n = n;
	m_x = new tFloat64[n + 1];
	m_y = new tFloat64[n + 1];
	m_b = new tFloat64[n + 1];
	m_c = new tFloat64[n + 1];
	m_d = new tFloat64[n + 1];

    // remember and shift base.
	for (tInt i = 1; i <= n; ++i) {
			m_x[i] = x[i - 1];
			m_y[i] = y[i - 1];
	}

  // linear interpolation when we have too little data (i.e. just two points!)
	if (n < 3) {
			m_b[2] = m_b[1] = (m_y[2] - m_y[1]) / (m_x[2] - m_x[1]);
			m_c[1] = m_c[2] = m_d[1] = m_d[2] = 0.0;
			return;
	}

	m_d[1] = m_x[2] - m_x[1];
	m_b[1] = - m_d[1];
	m_c[2] = (m_y[2] - m_y[1]) / m_d[1];
	m_c[1] = 0.0;
	for (tInt i = 2; i < n; ++i) {
	  m_d[i] = m_x[i + 1] - m_x[i];
	  m_b[i] = 2.0 * (m_d[i - 1] + m_d[i]);
	  m_c[i + 1] = (m_y[i + 1] - m_y[i]) / m_d[i];
	  m_c[i] = m_c[i + 1] - m_c[i];
	}
	
	m_b[n] = -m_d[n - 1];
	m_c[n] = 0.0;
	if (n != 3) {
		m_c[1] = m_c[3] / (m_x[4] - m_x[2]) - m_c[2] / (m_x[3] - m_x[1]);
		m_c[n] = m_c[n - 1] / (m_x[n] - m_x[n - 2]) - m_c[n - 2] / (m_x[n - 1] - m_x[n - 3]);
		m_c[1] *= m_d[1] * m_d[1] / (m_x[4] - m_x[1]);
		m_c[n] *= - m_d[n - 1] * m_d[n - 1] / (m_x[n] - m_x[n - 3]);
	}
	for (tInt i = 2; i < n; ++i) {
		tFloat64 T = m_d[i - 1] / m_b[i - 1];
		m_b[i] -= T * m_d[i - 1];
		m_c[i] -= T * m_c[i - 1];
	}

	m_c[n] /= m_b[n];
	for  (tInt i = n - 1; i > 0; --i)
			m_c[i] = (m_c[i] - m_d[i] * m_c[i + 1]) / m_b[i];

	m_b[n] = (m_y[n] - m_y[n - 1]) / m_d[n - 1] + m_d[n - 1] * (m_c[n - 1] + 2.0 * m_c[n]);
	
	for (tInt i = 1; i < n; ++i) {
	  m_b[i] = (m_y[i + 1] - m_y[i]) / m_d[i] - m_d[i] * (m_c[i + 1] + 2.0 * m_c[i]);
	  m_d[i] = (m_c[i + 1] - m_c[i]) / m_d[i];
	  m_c[i] *= 3.0;
	}
	m_c[n] *= 3.0;
	m_d[n] = m_d[n - 1];
}

//! m_class destructor
~Cubic()
{
  delete [] m_x;
  delete [] m_y;
  delete [] m_b;
  delete [] m_c;
  delete [] m_d;
}

//! Returns an interpolated value.

tFloat64 getValue(tFloat64 x)
{
   if (x <= m_x[1])
      return (((m_y[2]-m_y[1])/(m_x[2]-m_x[1]))*(x-m_x[1])+m_y[1]);
   if (x >= m_x[m_n]) {
//       LOG_INFO(cString::Format(" x = %f  mx[1] %f   mx[2] %f   my[1] %f    my[2] %f   -> %f",x,m_x[m_n],m_x[m_n-1],m_y[m_n],m_y[m_n-1],(((m_y[m_n]-m_y[m_n-1])/(m_x[m_n]-m_x[m_n-1]))*(x-m_x[m_n-1])+m_y[m_n-1])));
      return (((m_y[m_n]-m_y[m_n-1])/(m_x[m_n]-m_x[m_n-1]))*(x-m_x[m_n-1])+m_y[m_n-1]);
   }

   if (x <= m_x[2])
   {
     tFloat64 dx = x - m_x[1];
     return m_y[1] + dx * (m_b[1] + dx * (m_c[1] + dx * m_d[1]));
   }

   tInt i = 1, j = m_n + 1;
   do {
     tInt k = (i + j) / 2;
     (x < m_x[k]) ? j = k : i = k;
   } while (j > i + 1);

   tFloat64 dx = x - m_x[i];
   return m_y[i] + dx * (m_b[i] + dx * (m_c[i] + dx * m_d[i]));
}

bool isInRange(tFloat64 x)
{
    if(m_rangeStart <= x && x <= m_rangeStop)
        return true;
    return false;
}

private:

tInt m_n;
tFloat32 m_rangeStart, m_rangeStop;
tFloat64 *m_x, *m_y, *m_b, *m_c, *m_d;
};

//
////! A static function implementing the Cubic Class for one off calculations
//
//tFloat64 Cubic_once(tInt N, vector<tFloat32> x, vector<tFloat32> y, tFloat64 a )
//{
//  // This function is created to enable an Instant Calculator on CodeCogs. 
//  // You probably shouldn't be using this function otherwise. 
//
//  Cubic A(N, x, y);
//   return A.getValue(a);
//}



#endif

