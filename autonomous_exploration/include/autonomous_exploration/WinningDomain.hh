/*
 * WinningDomain.hh
 *
 *  created: Jan 2017
 *   author: Matthias Rungger
 *              
 */

/** @file **/

#ifndef WINNINGDOMAIN_HH_
#define WINNINGDOMAIN_HH_

#include <vector>
#include <iostream>
#include <limits>

/* to get abs_type alias */
#include "UniformGrid.hh"

/** @namespace scots **/ 
namespace scots {

/**
 * @class WinningDomain
 *
 * @brief The abstract states from which the controller is winning
 *
 * The WinningDomain is computed by one of the game solving algorithms in
 * GameSovler.hh and represents the set of abstract states in {0,...,N-1} 
 * from which the controller is winning.
 *
 * For a mathematical definition see the <a href="./../../manual/manual.pdf">manual.pdf</a>.
 *
 *
 **/

class WinningDomain {
/* allow the write_to_file function to access the private members */
friend bool write_to_file(const WinningDomain&, const std::string&, bool);
private:
  /* size of state alphabet N */
  abs_type m_no_states;
  /* size of input alphabet M */
  abs_type m_no_inputs;
  /*
   * array of size N  \n
   * (m_winning_domain[i]=m_loosing if i is not winning) 
   */
  std::vector<abs_type> m_winning_domain;
  /*
   * bool array of size N*M encoding the valid inputs\n
   * m_inputs[i*M +j]==true iff j is a valid input at i) 
   */
  std::vector<bool> m_inputs; 
  /*
   * constant that is used to indicate that a sate is not in 
   * the winning domain; not intended to be chaned 
   */
  abs_type m_loosing;

public:
  /** @cond  EXCLUDE from doxygen **/
  /* default constructor */
  WinningDomain() : m_no_states(0),
                    m_no_inputs(0),
                    m_winning_domain {},
                    m_inputs {},
                    m_loosing(std::numeric_limits<abs_type>::max()) {} 
  /* destructor */
  ~WinningDomain()=default;
  /* copy constructor  */
  WinningDomain(const WinningDomain&)=default;
  /* move constructor */
  WinningDomain(WinningDomain&&)=default;
  /* copy assignment operator */
  WinningDomain& operator=(const WinningDomain&)=default; 
  /* move assignment operator */
  WinningDomain& operator=(WinningDomain&&)=default;
  /* @endcond */

  /** @brief construct WinningDomain with number of states and number of abstract inputs **/
  WinningDomain(abs_type no_states,
                abs_type no_inputs,
                abs_type loosing=std::numeric_limits<abs_type>::max()) : 
                m_no_states(no_states), 
                m_no_inputs(no_inputs),
                m_loosing(loosing) {}

  /** @brief construct WinningDomain with array of winning states **/
  WinningDomain(abs_type no_states,
                abs_type no_inputs,
                std::vector<abs_type>&& winning_domain,
                abs_type loosing=std::numeric_limits<abs_type>::max()) : 
                m_no_states(no_states), 
                m_no_inputs(no_inputs),
                m_winning_domain(std::move(winning_domain)),
                m_loosing(loosing) {}

  /** @brief construct WinningDomain with array of winning states and valid inputs **/
  WinningDomain(abs_type no_states,
                abs_type no_inputs,
                std::vector<abs_type>&& winning_domain,
                std::vector<bool>&& inputs,
                abs_type loosing=std::numeric_limits<abs_type>::max()) : 
                m_no_states(no_states),
                m_no_inputs(no_inputs),
                m_winning_domain(std::move(winning_domain)),
                m_inputs(std::move(inputs)),
                m_loosing(loosing) {}

  /** @brief check if state i is winning **/
  bool is_winning(const abs_type& i) const {
    if(i<m_winning_domain.size() && m_winning_domain[i]!=m_loosing) {
      return true;
    }
    return false;
  }

  /** @brief return valid inputs associated with state i **/
  std::vector<abs_type> get_inputs(const abs_type& i) const {
    std::vector<abs_type> inputs{};
    /* extract input information from m_inputs matrix */
    if(m_inputs.size()==m_no_states*m_no_inputs) {
      for(abs_type j=0; j<m_no_inputs; j++) {
        if(m_inputs[i*m_no_inputs+j]) {
          inputs.push_back(j);
        }
      }
      return inputs;
    }
    /* otherwise valid input might be written directly in m_winning_domain */
    if(is_winning(i)) {
      inputs.push_back(m_winning_domain[i]);
      return inputs;
    }
    return inputs;
  }

  /** @brief get number of winning states (=size of winning domain) **/
  abs_type get_size() const {
    abs_type count=0;
    for(abs_type i=0; i<m_no_states; i++) {
      if(is_winning(i)) {
        count++;
      }
    }
    return count;
  }
  /** @brief get the size of the state alphabet **/
  abs_type get_no_states() const {
    return m_no_states;
  }

  /** @brief get the size of the input alphabet **/
  abs_type get_no_inputs() const {
    return m_no_inputs;
  }

  /** @brief get winning states **/
  std::vector<abs_type> get_winning_domain() const {
    std::vector<abs_type> ws{};
    for(abs_type i=0; i<m_no_states; i++) {
      if(is_winning(i)) {
        ws.push_back(i);
      }
    }
    return ws;
  }

};

} /* close namespace */
#endif /* WINNINGDOMAIN_HH_ */
