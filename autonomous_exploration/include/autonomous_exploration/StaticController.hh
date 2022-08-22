/*
 * StaticController.hh
 *
 *  created: Dec 2016
 *   author: Frederik Kunik
 *           Matthias Rungger
 *           
 */

/** @file **/

#ifndef STATICCONTROLLER_HH_
#define STATICCONTROLLER_HH_

#include <vector>
#include <array>
#include <exception>

#include "UniformGrid.hh"
#include "WinningDomain.hh"

/** @namespace scots **/ 
namespace scots{
/**
 * @brief StaticController class to simualte the closed loop
 **/
class StaticController {
/* allow the write_to_file function to access the private members */
friend bool write_to_file(const StaticController&, const std::string&,bool);
private:
  UniformGrid m_input_grid;
  UniformGrid m_state_grid;

  WinningDomain m_winning_domain;

public:
  /** @cond  EXCLUDE from doxygen **/
  /* default constructor */
  StaticController()=default;                      
  /* destructor */
  ~StaticController()=default;
  /* copy constructor  */
  StaticController(const StaticController& other) = default;
  /* move constructor */
  StaticController(StaticController&&)=default;
  /* copy assignment operator */
  StaticController& operator=(const StaticController&)=default;
  /* move assignment operator */
  StaticController& operator=(StaticController&&)=default;
  /* @endcond */

  /** @brief controller constructor **/
  StaticController(const UniformGrid& state_grid,
                   const UniformGrid& input_grid,
                   WinningDomain&& winning_domain) :
                   m_input_grid(input_grid),
                   m_state_grid(state_grid),
                   m_winning_domain(std::move(winning_domain)) { }

  /** @brief get a std::vector containing the valid control inputs at state x \n
    * does throw a runtime error if state x is out of winning domain**/
  template<class state_type, class input_type>
  std::vector<input_type> get_control(const state_type &x) const {
    /* abstract state index */
    abs_type i = m_state_grid.xtoi(x);
    std::vector<abs_type> abs_inputs = m_winning_domain.get_inputs(i);

    if(!m_winning_domain.is_winning(i)) {
      std::ostringstream os;
      os << "\nscots::StaticController: state ";
      for(int i=0; i<m_state_grid.get_dim(); i++) {
        os << x[i] << " ";
      }
      os << "is out of winning domain: no progress possible.";
      throw std::runtime_error(os.str().c_str());
    }
    std::vector<input_type> inputs{};
    for(abs_type i=0; i<abs_inputs.size(); i++) {
      input_type u;
      m_input_grid.itox(abs_inputs[i],u);
      inputs.push_back(u);
    }
    return inputs;
  }

  /** @brief get a std::vector containing the valid control inputs at state x \n
    * does return an empty vector if state is out of winning domain**/
  template<class state_type, class input_type>
  std::vector<input_type> peek_control(const state_type &x) const {
    /* abstract state index */
    abs_type i = m_state_grid.xtoi(x);

    if(!m_winning_domain.is_winning(i)) {
        return std::vector<input_type>{};
    }
    std::vector<abs_type> abs_inputs = m_winning_domain.get_inputs(i);
    return m_input_grid.ItoX<input_type>(abs_inputs);
  }

  /** @brief get a std::vector containing the states in the winning domain **/
  template<class state_type>
  std::vector<state_type> get_domain() const {
    /* abstract state indices */
    std::vector<abs_type> abs_domain = m_winning_domain.get_winning_domain();

    /* convert to cell centers */
    std::vector<std::vector<double>> domain(abs_domain.size());
    for(abs_type i=0; i<abs_domain.size(); i++) {
      state_type x;
      m_state_grid.itox(abs_domain[i],x);
      domain[i]=x;
    }
    return domain;
  }
};

} /* end of namespace scots */
#endif /* STATICCONTROLLER_HH_ */
