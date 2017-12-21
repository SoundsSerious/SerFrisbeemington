// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           �cole Polytechnique de Montr�al
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef KFILTER_IMPL_HPP
#define KFILTER_IMPL_HPP

#include "kalman/kfilter.h"

//! \internal
//! Flag : \a n has changed
#define KALMAN_N_MODIFIED    1

//! \internal
//! Flag : \a nu has changed
#define KALMAN_NU_MODIFIED  (1<<1)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NW_MODIFIED  (1<<2)

//! \internal
//! Flag : \a m has changed
#define KALMAN_M_MODIFIED   (1<<3)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NV_MODIFIED  (1<<4)

//! \internal
//! Flag : \a P has changed
#define KALMAN_P_MODIFIED   (1<<5)

//! \internal
//! Mask : used to reset dimension flags
#define KALMAN_LOWMASK      ((1<<8) - 1)

//! \internal
//! Flag : \a A has changed
#define KALMAN_A_MODIFIED   (1<<8)

//! \internal
//! Flag : \a W has changed
#define KALMAN_W_MODIFIED   (1<<9)

//! \internal
//! Flag : \a Q has changed
#define KALMAN_Q_MODIFIED   (1<<10)

//! \internal
//! Mask : used to reset time update matrix flags
#define KALMAN_MIDMASK      ( ((1<<4) - 1) << 8 )

//! \internal
//! Flag : \a H has changed
#define KALMAN_H_MODIFIED   (1<<12)

//! \internal
//! Flag : \a V has changed
#define KALMAN_V_MODIFIED   (1<<13)

//! \internal
//! Flag : \a R has changed
#define KALMAN_R_MODIFIED   (1<<14)

//! \internal
//! Mask : used to reset measure update matrix flags
#define KALMAN_HIGHMASK     ( ((1<<4) - 1) << 12 )

//! \file
//! \brief Contains the implementation of the \c KFilter base template class.

namespace Kalman {

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  KFilter<T, BEG, OQ, OVR, DBG>::~KFilter() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeBaseB() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeB() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeProcess() {

    // x = Ax + Bu + Ww    n.1 = n.n * n.1 + n.nu * nu.1
    makeB();

    K_UINT_32 i, j;
    x__.resize(n);

    for (i = BEG; i < n + BEG; ++i) {

      x__(i) = T(0.0);

      for (j = BEG; j < n + BEG; ++j)
        x__(i) += A(i,j) * x(j);

      for (j = BEG; j < nu + BEG; ++j)
        x__(i) += B(i,j) * u(j);

    }

    x.swap(x__);

  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeMeasure() {

    // z = Hx + Vv
    K_UINT_32 i, j;

    z.resize(m);
    for (i = BEG; i < m + BEG; ++i) {

      z(i) = T(0.0);

      for (j = BEG; j < n + BEG; ++j)
        z(i) += H(i,j) * x(j);

    }

  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::sizeUpdate() {

    if (flags & ( KALMAN_N_MODIFIED | KALMAN_NU_MODIFIED ) ) {
      B.resize(n, nu);
      makeBaseB();
    }

    EKFilter<T, BEG, OQ, OVR, DBG>::sizeUpdate();
  }

}

#endif
