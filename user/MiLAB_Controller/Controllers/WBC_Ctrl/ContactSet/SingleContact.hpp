#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }
  void setFootStateDes(const DVec<T>& pos, const DVec<T>& vel);
  void setKpKd(const DVec<T>& Kp, const DVec<T>& Kd);

 protected:
  T _max_Fz;
  int _contact_pt;
  int _dim_U;

  DVec<T> _pos_des;
  DVec<T> _vel_des;
  DVec<T> _Kp;
  DVec<T> _Kd;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
