/*
 * RungeKutta4.hh
 *
 *  created on: 22.04.2015
 *      author: rungger
 */

/** @file **/
#ifndef RUNGEKUTTA4_HH_
#define RUNGEKUTTA4_HH_

/** @namespace scots **/ 
namespace scots {

/**
 *  
 * @brief Fixed step size ODE solver implementing a RungeKutta scheme of order 4 
 * 
 * @param rhs - lambda expression implementing the rhs of the ode
 *              \f[ \dot \xi(t) = rhs(\xi(t),u), \xi(0)=x \f]
 *              with signature 
 *              \verbatim [] (state_type& xx,  const state_type &x, const input_type &u) ->  void \endverbatim
 * @param x - state_type initial state x
 * @param u - input_type constant input u
 * @param dim - state space dimension
 * @param tau - sampling time
 * @param nint - number of intermediate steps (default = 10)
 * @return the solution of IVP at time tau \f$ \xi(\tau) \f$ stored in x
 **/
template<class RHS, class state_type, class input_type>
void runge_kutta_fixed4(RHS rhs, state_type &x, input_type &u, const int dim, const double tau, const int nint=10) noexcept {
  state_type k[4];
  state_type tmp;

  double h=tau/(double)nint;

  for(int t=0; t<nint; t++) {
    rhs(k[0],x,u);
    for(int i=0;i<dim;i++)
      tmp[i]=x[i]+h/2*k[0][i];

    rhs(k[1],tmp, u);
    for(int i=0;i<dim;i++)
      tmp[i]=x[i]+h/2*k[1][i];

    rhs(k[2],tmp, u);
    for(int i=0;i<dim;i++)
      tmp[i]=x[i]+h*k[2][i];

    rhs(k[3],tmp, u);
    for(int i=0; i<dim; i++)
      x[i] = x[i] + (h/6)*(k[0][i] + 2*k[1][i] + 2*k[2][i] + k[3][i]);
  }
}

} /* close namespace */

#endif /* RUNGEKUTTA4_HH_ */
