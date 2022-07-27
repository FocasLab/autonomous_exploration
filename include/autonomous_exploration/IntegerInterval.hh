/*
 * IntegerInterval.hh
 *
 *  created: Jan 2016
 *   author: Matthias Rungger
 */

/** @file **/

#ifndef INTEGERINTERVAL_HH_
#define INTEGERINTERVAL_HH_


#include <iostream>
#include <vector>
#include <memory>

/* cudd library */
#include "cuddObj.hh"

namespace scots {

/** @class IntegerInterval 
 *
 *  @brief A class to represent the integers in a closed integer interval [lb; ub] as BDDs. \n
 *  The least significant bit is the BDD variable with the highest ID number.\n
 *  The most significant bit is the BDD variable with the lowest ID number.
 * 
 **/
template<class int_type >
class IntegerInterval {
private:
  /* lower bound of interval */
  int_type m_lb;
  /* upper bound of interval */
  int_type m_ub;
  /* number of integers in the interval */
  int_type m_size;
  /* m_bdd_vars contains the BDD variables */
  std::vector<BDD> m_bdd_vars;
  /* m_bdd_var_id contains the BDD variable IDs */
  std::vector<unsigned int> m_bdd_var_id;
  /* an array[m_size] containing the BDD representation of each element */
  std::vector<BDD> m_int_to_bdd;
  /* helper function */
  void add_one(int* phase, unsigned int num) {
    int carry = 1;
    for(int i=num-1; carry && (i>=0) ;i--) {
      if(phase[i]) {
        phase[i]=0;
        carry=1;
      } else {
        phase[i]=1;
        carry=0;
      }
    }
  }

public:
  /* @cond  EXCLUDE from doxygen */
  /* default constructor */
  IntegerInterval() : m_lb(0),  
                      m_ub(0), 
                      m_bdd_vars{},
                      m_bdd_var_id{},
                      m_int_to_bdd{} { }
  /* @endcond */
    
  /** @brief Instantiate the IntegerInterval with the integer interval [lb;ub]\n 
   *  optionally, provide the bdd variable ids to reprsent the integer interval **/
  IntegerInterval(const Cudd& manager,
                  int_type lb, int_type ub,
                  const std::vector<unsigned int>& bdd_var_id = {}) :
                  m_lb(lb), m_ub(ub), m_bdd_var_id(bdd_var_id) {
    /* compute necessar no of BDD variables to represent the integer in the interval  */ 
    m_size = ub-lb+1;
    unsigned int no_bdd_var = m_bdd_var_id.size();
    if(!no_bdd_var) {
      int_type x = ub-lb;
      while(x) {
        x>>=1u;
        no_bdd_var++;
      }
      if(!no_bdd_var) {
        no_bdd_var=1;
      }
    }
    /* get new BDD variable IDs */
    m_bdd_vars.resize(no_bdd_var);
    m_bdd_var_id.resize(no_bdd_var);
    std::unique_ptr<BDD[]> vars{ new BDD[no_bdd_var] }; 
    for(unsigned int j=0; j<no_bdd_var; j++) {
      /* copy ids from bdd_var_id or create new ones */
      if(!bdd_var_id.size()) {
        /* get new IDs */
        vars[j] = manager.bddVar();
        m_bdd_var_id[j]=vars[j].NodeReadIndex();
      } else {
        /* copy IDs from bdd_var_id */
        vars[j] = manager.bddVar(bdd_var_id[j]);
      }
      m_bdd_vars[j]=vars[j];
    }
    /* create BDDs to represent the integers in the interval */
    std::unique_ptr<int[]> phase{ new int[no_bdd_var] () }; 
    m_int_to_bdd.resize(m_size);
    for(int_type i=0; i<m_size; i++) {
      m_int_to_bdd[i] = manager.bddComputeCube(vars.get(),phase.get(),no_bdd_var);
      add_one(phase.get(),no_bdd_var);
      //m_int_to_bdd[i].PrintMinterm();
    }
  }

  /** @brief maps an integer in the interval to its BDD representation **/
  BDD int_to_bdd(int_type i) const {
    return m_int_to_bdd[i-m_lb];
  }

  /** @brief maps a (sub)interval in [lb; ub]  to its BDD representation **/
  BDD interval_to_bdd(const Cudd& manager, int_type lb, int_type ub) const {
    return manager.Interval(m_bdd_vars, static_cast<unsigned int>(lb)-m_lb, static_cast<unsigned int>(ub)-m_lb);
  }

  /** @brief prints the BDD variable IDs used to represent the integer interval */
  void print_bdd_IDs() const {
    std::cout << "BDD variable IDs: ";
    for(size_t j=0; j<m_bdd_var_id.size(); j++)  {
      std::cout << m_bdd_var_id[j] << " ";
    }
    std::cout << "\n";
  }
 
  /** @brief get a vector with the BDD variables used to represent the integer interval **/
  std::vector<BDD> get_bdd_vars() const {
    return m_bdd_vars;
  }

  /** @brief get a vector with the BDD variable IDs used to represent the integer interval **/
  std::vector<unsigned int> get_bdd_var_ids() const {
    return m_bdd_var_id;
  }

  /** @brief get number of BDD variables used to represent the integer interval **/
  int_type get_no_bdd_vars() const {
    return m_bdd_vars.size();
  }

  /** @brief get a BDD that encodes all elements of the integer interval **/
  BDD get_all_elements() const {
    BDD elements = m_int_to_bdd[0];
    for(const auto& bdd : m_int_to_bdd)
      elements = elements | bdd;
    return elements;
  }

  /** @brief get a vector with strings for BDD variable names\n as used in slugs for integer intervals 
   *
   *  For example, the integer interval variable x:0...4 declared in slugs is encoded by 3 BDD variables
   *  with names (from LSB to MSB) x@0.0.4, x@1, x@2
   *  
   **/
  std::vector<std::string> get_slugs_var_names() const {
    auto no_bdd_var=m_bdd_vars.size();
    std::vector<std::string> var_names(no_bdd_var);
    var_names[no_bdd_var-1]=std::string{"@0."}+std::to_string(m_lb)+std::string{"."}+std::to_string(m_ub);
    for(size_t i=0; i<(no_bdd_var-1); i++) {
      var_names[i]=std::string{"@"}+std::to_string((no_bdd_var-i-1));
    }
    return var_names;
  }

}; /* close class def */
} /* close namespace */
#endif /* INTEGERINTERVAL_HH_ */
