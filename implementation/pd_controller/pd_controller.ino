void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  


}


// in: F_ref (from serial), F (from load cell)
// need: F_err, d/dt(F_err), tau_m_adj = k_p*F_err + k_d*d/dt(F_err)
// tau_m = tau_m+tau_m_adj


double control(const std::vector<float>& F, const std::vector<float>& F_prev, const std::vector<float>& F_ref, const std::vector<float>& F_ref_prev, double dt, const std::vector<float>& Tau_m, double k_p, double k_d){
  double F_err = F_ref - F;
  double F_err_prev = F_ref - F_prev;
  double F_err_dot = (F_err-F_err_prev)/dt;

  Tau_m_adj = k_p*F_err + k_d*F_err_dot;

  return (Tau_m+Tau_m_adj)
}