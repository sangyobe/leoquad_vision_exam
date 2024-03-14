namespace dtCore {

////////////////////////////////////////////////////////////////////////////////
// Implementation of dtBezierTrajectory
//

/*! \details Initialize without input parameter
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory()
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");

    memset(m_pi, 0, sizeof(ValueType) * m_dof);
    memset(m_pf, 0, sizeof(ValueType) * m_dof);
    memset(m_vi, 0, sizeof(ValueType) * m_dof);
    memset(m_vf, 0, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    memset(m_pc, 0, sizeof(ValueType) * m_dof * (m_maxNum + 6));
}

/*! \details Initialize and configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [vi(init velocity), vf(target velocity), ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                   const ContRefType pi, const ContRefType pf,
                                                                   const ContRefType pc, const uint16_t pcNum,
                                                                   const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memset(m_vi, 0, sizeof(ValueType) * m_dof);
    memset(m_vf, 0, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }

    m_inputType = POS;
    Configure();
}

/*! \details Initialize and configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                  const ContRefType pi, const ContRefType pf, 
                                                                  const ContRefType vi, const ContRefType vf, 
                                                                  const ContRefType pc, const uint16_t pcNum,
                                                                  const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }

    m_inputType = VEL;
    Configure();
}

/*! \details Initialize and configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] ai init acceleration (x/sec^2)
    \param[in] af target acceleration (x/sec^2)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                  const ContRefType pi, const ContRefType pf, 
                                                                  const ContRefType vi, const ContRefType vf, 
                                                                  const ContRefType ai, const ContRefType af, 
                                                                  const ContRefType pc, const uint16_t pcNum,
                                                                  const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memcpy(m_ai, ai, sizeof(ValueType) * m_dof);
    memcpy(m_af, af, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }

    m_inputType = ACC;
    Configure();    
}

/*! \details Initialize and configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [vi(init velocity), vf(target velocity), ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                   const ContRefType pi, const ContRefType pf,
                                                                   const ContRefType pc[m_dof], const uint16_t pcNum,
                                                                   const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memset(m_vi, 0, sizeof(ValueType) * m_dof);
    memset(m_vf, 0, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }

    m_inputType = POS;
    Configure();
}

/*! \details Initialize and configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                  const ContRefType pi, const ContRefType pf, 
                                                                  const ContRefType vi, const ContRefType vf, 
                                                                  const ContRefType pc[m_dof], const uint16_t pcNum,
                                                                  const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }

    m_inputType = VEL;
    Configure();
}

/*! \details Configure control points and coefficients of the bezier from the parameters entered.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] ai init acceleration (x/sec^2)
    \param[in] af target acceleration (x/sec^2)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::dtBezierTrajectory(const ValueType duration, 
                                                                  const ContRefType pi, const ContRefType pf, 
                                                                  const ContRefType vi, const ContRefType vf, 
                                                                  const ContRefType ai, const ContRefType af, 
                                                                  const ContRefType pc[m_dof], const uint16_t pcNum,
                                                                  const ValueType timeOffset)
: m_duration(duration), m_ti(timeOffset), m_pcNum(pcNum)
{
    static_assert(m_maxNum > 0, "Invalid degree of bezier.");
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memcpy(m_ai, ai, sizeof(ValueType) * m_dof);
    memcpy(m_af, af, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }

    m_inputType = ACC;
    Configure();
}

template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
dtBezierTrajectory<ValueType, m_dof, m_maxNum>::~dtBezierTrajectory() {}

/*! \details Calculates the desired position(p) corresponding to the time(t) entered. 
    \param[in] t current time (sec)
    \param[out] p desired position (x)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::Interpolate(const ValueType t, ContRefType p) const 
{
    ValueType t_ = t - this->m_ti;
    if (t_ < 0) 
    {
        // before trajectory start
        memcpy(p, this->m_pi, sizeof(ValueType) * m_dof);
    } 
    else if (t_ > this->m_duration) 
    {
        // after trajectory end
        memcpy(p, this->m_pf, sizeof(ValueType) * m_dof);
    } 
    else 
    {
        for (uint16_t i = 0; i < m_dof; i++) 
        {
            m_interpolator[i].Interpolate(t_, p[i]);
        }
    }
}

/*! \details Calculates the desired position(p) and velocity(v) corresponding to the time(t) entered. 
    \param[in] t current time (sec)
    \param[out] p desired position (x)
    \param[out] v desired velocity (x/sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::Interpolate(const ValueType t, ContRefType p, ContRefType v) const 
{
    ValueType t_ = t - this->m_ti;
    if (t_ < 0) 
    {
        // before trajectory start
        memcpy(p, this->m_pi, sizeof(ValueType) * m_dof);
        memset(v, 0, sizeof(ValueType) * m_dof);
    } 
    else if (t_ > this->m_duration) 
    {
        // after trajectory end
        memcpy(p, this->m_pf, sizeof(ValueType) * m_dof);
        memset(v, 0, sizeof(ValueType) * m_dof);
    } 
    else 
    {
        for (uint16_t i = 0; i < m_dof; i++) 
        {
            m_interpolator[i].Interpolate(t_, p[i], v[i]);
        }
    }
}

/*! \details Calculates the desired position(p), velocity(v) and acceleration(a) corresponding to the time(t) entered. 
    \param[in] t current time (sec)
    \param[out] p desired position (x)
    \param[out] v desired velocity (x/sec)
    \param[out] a desired acceleration (x/sec^2)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::Interpolate(const ValueType t, ContRefType p, ContRefType v, ContRefType a) const 
{
    ValueType t_ = t - this->m_ti;
    if (t_ < 0) 
    {
        // before trajectory start
        memcpy(p, this->m_pi, sizeof(ValueType) * m_dof);
        memset(v, 0, sizeof(ValueType) * m_dof);
        memset(a, 0, sizeof(ValueType) * m_dof);
    } 
    else if (t_ > this->m_duration) 
    {
        // after trajectory end
        memcpy(p, this->m_pf, sizeof(ValueType) * m_dof);
        memset(v, 0, sizeof(ValueType) * m_dof);
        memset(a, 0, sizeof(ValueType) * m_dof);
    } 
    else 
    {
        for (uint16_t i = 0; i < m_dof; i++) 
        {
            m_interpolator[i].Interpolate(t_, p[i], v[i], a[i]);
        }
    }
}

/*! \details Configure the coefficients of polynomial through parameters entered from functions below 
             (SetParam, SetDuration, SetInitParam, SetTargetParam, SetControlParam, SetTimeOffset).
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::Configure() 
{
    switch (m_inputType)
    {
        case POS:
        {
            for (uint16_t i = 0; i < m_dof; i++) 
            {
                m_interpolator[i].Configure(this->m_pi[i], this->m_pf[i], this->m_pc[i], m_pcNum, this->m_duration);
            }
            break;
        }
        case VEL:
        {
            for (uint16_t i = 0; i < m_dof; i++) 
            {
                m_interpolator[i].Configure(this->m_pi[i], this->m_pf[i], this->m_vi[i], this->m_vf[i], this->m_pc[i], m_pcNum, this->m_duration);
            }
            break;
        }
        case ACC:
        {
            for (uint16_t i = 0; i < m_dof; i++) 
            {
                m_interpolator[i].Configure(this->m_pi[i], this->m_pf[i], this->m_vi[i], this->m_vf[i], this->m_ai[i], this->m_af[i], this->m_pc[i], m_pcNum, this->m_duration);
            }
            break;
        }
    }
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [vi(init velocity), vf(target velocity), ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType pc, const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memset(m_vi, 0, sizeof(ValueType) * m_dof);
    memset(m_vf, 0, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }
    m_pcNum = pcNum;
    m_inputType = POS;
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType vi, const ContRefType vf, 
                                                             const ContRefType pc, const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }
    m_pcNum = pcNum;
    m_inputType = VEL;
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] vi init acceleration (x/sec^2)
    \param[in] vf target acceleration (x/sec^2)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType vi, const ContRefType vf, 
                                                             const ContRefType ai, const ContRefType af, 
                                                             const ContRefType pc, const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memcpy(m_ai, ai, sizeof(ValueType) * m_dof);
    memcpy(m_af, af, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }
    m_pcNum = pcNum;
    m_inputType = ACC;
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [vi(init velocity), vf(target velocity), ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType pc[m_dof], const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memset(m_vi, 0, sizeof(ValueType) * m_dof);
    memset(m_vf, 0, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }
    m_pcNum = pcNum;
    m_inputType = POS;
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
    The parameters [ai(init acceleration), af(target acceleration)] that are not entered are set to zero.
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType vi, const ContRefType vf, 
                                                             const ContRefType pc[m_dof], const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memset(m_ai, 0, sizeof(ValueType) * m_dof);
    memset(m_af, 0, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }
    m_pcNum = pcNum;  
    m_inputType = VEL;
}

/*! \details Enter parameters for the Configure() function.
    \param[in] duration trajectory duration (sec)
    \param[in] pi init position (x)
    \param[in] pf target position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] vf target velocity (x/sec)
    \param[in] vi init acceleration (x/sec^2)
    \param[in] vf target acceleration (x/sec^2)
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
    \param[in] timeOffset trajectory delay (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetParam(const ValueType duration, 
                                                             const ContRefType pi, const ContRefType pf, 
                                                             const ContRefType vi, const ContRefType vf, 
                                                             const ContRefType ai, const ContRefType af, 
                                                             const ContRefType pc[m_dof], const uint16_t pcNum,
                                                             const ValueType timeOffset)
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");

    m_duration = duration;
    m_ti = timeOffset;
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
    memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
    memcpy(m_ai, ai, sizeof(ValueType) * m_dof);
    memcpy(m_af, af, sizeof(ValueType) * m_dof);
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }
    m_pcNum = pcNum;
    m_inputType = ACC;
}

/*! \details  Enter trajectory duration for the Configure() function.
    \param[in] duration trajectory duration (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetDuration(const ValueType duration) 
{
    assert(duration > m_tolerance && "Trajectory duration should be greater than zero");
    m_duration = duration;
}

/*! \details Enter init parameter for the Configure() function.
    \param[in] pi init position (x)
    \param[in] vi init velocity (x/sec)
    \param[in] ai init acceleration (x/sec^2)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetInitParam(const ContRefType pi, const ContRefType vi, const ContRefType ai)
{
    memcpy(m_pi, pi, sizeof(ValueType) * m_dof);
    m_inputType = POS;
    if (vi)
    {
        memcpy(m_vi, vi, sizeof(ValueType) * m_dof);
        m_inputType = VEL;
    }
    else
    {
        memset(m_vi, 0, sizeof(ValueType) * m_dof);
    }
    if (ai)
    {
        memcpy(m_ai, ai, sizeof(ValueType) * m_dof);
        m_inputType = ACC;
    }
    else
    {
        memset(m_ai, 0, sizeof(ValueType) * m_dof);
    }
}

/*! \details Enter target parameter for the Configure function.
    \param[in] pf target position (x)
    \param[in] vf target velocity (x/sec)
    \param[in] af target acceleration (x/sec^2)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetTargetParam(const ContRefType pf, const ContRefType vf, const ContRefType af)
{
    memcpy(m_pf, pf, sizeof(ValueType) * m_dof);
    m_inputType = POS;
    if (vf)
    {
        memcpy(m_vf, vf, sizeof(ValueType) * m_dof);
        m_inputType = VEL;
    }
    else
    {
        memset(m_vf, 0, sizeof(ValueType) * m_dof);
    }
    if (af)
    {
        memcpy(m_af, af, sizeof(ValueType) * m_dof);
        m_inputType = ACC;
    }
    else
    {
        memset(m_af, 0, sizeof(ValueType) * m_dof);
    }
}

/*! \details Enter input contorl parameter for the Configure() function.
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetControlParam(const ContRefType pc, const uint16_t pcNum) 
{
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[m_dof*j + i]; 
        }
    }
    m_pcNum = pcNum;
}

/*! \details Enter input contorl parameter for the Configure() function.
    \param[in] pc input control point (x)
    \param[in] pcNum number of input control point
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetControlParam(const ContRefType pc[m_dof], const uint16_t pcNum) 
{
    assert(m_maxNum >= pcNum && "input control point num should be smaller than m_maxNum(max input control point num)");
    // Transpose input control point.
    for (uint16_t i = 0; i < m_dof; i++)
    {
        for (uint16_t j = 0; j < pcNum; j++)
        {
            m_pc[i][j] = pc[j][i]; 
        }
    }
    m_pcNum = pcNum;
}

/*! \details Enter trajectory delay for the Configure() function.
    \param[in] timeOffset trajectory delay (sec)
*/
template <typename ValueType, uint16_t m_dof, uint16_t m_maxNum>
void dtBezierTrajectory<ValueType, m_dof, m_maxNum>::SetTimeOffset(const ValueType timeOffset) 
{
    m_ti = timeOffset;
}

} // namespace dtCore