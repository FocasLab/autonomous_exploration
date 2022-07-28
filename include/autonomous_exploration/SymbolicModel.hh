/*
 * SymbolicModel.hh
 *
 *  created: Jan 2017
 *   author: Matthias Rungger
 */

/** @file **/
#ifndef SYMBOLICMODEL_HH_
#define SYMBOLICMODEL_HH_

#include <iostream>
#include <vector>


#include "SymbolicSet.hh"

/** @namespace scots **/ 
namespace scots {
/**
 * @class SymbolicModel
 * 
 * @brief Compute the transition function of a symbolic model as BDD using the growth bound. 
 *
 * See 
 * - the manual in <a href="./../../manual/manual.pdf">manual.pdf</a>
 * - http://arxiv.org/abs/1503.03715 for theoretical background 
 *
 **/
template<class state_type, class input_type>
class SymbolicModel {
private:
  /* print progress to the console (default m_verbose=true) */
  bool m_verbose=true;
  /* SymbolicSet conaining the BDD vars of the pre  */
  const SymbolicSet m_pre;
  /* SymbolicSet conaining the BDD vars of the inputs  */
  const SymbolicSet m_input;
  /* SymbolicSet conaining the BDD vars of the post  */
  const SymbolicSet m_post;
  /* measurement error bound */
  std::unique_ptr<double[]> m_z;

  void progress(const abs_type& i, const abs_type& N, abs_type& counter) {
    if(!m_verbose)
      return;
    if(((double)i/(double)N*100)>counter){
      if(counter==0)
        std::cout << "loop: ";
      if((counter%10)==0)
        std::cout << counter;
      else if((counter%2)==0) 
        std::cout << ".";
      counter++;
    }
    std::flush(std::cout); 
    if(i==(N-1))
      std::cout << "100\n";
  }

public:
  /* @cond  EXCLUDE from doxygen*/
  /* destructor */
  ~SymbolicModel() = default;
  /* deactivate standard constructor */
  SymbolicModel() = delete;
  /* cannot be copied or moved */
  SymbolicModel(SymbolicModel&&) = delete;
  SymbolicModel(const SymbolicModel&) = delete;
  SymbolicModel& operator=(SymbolicModel&&)=delete;
  SymbolicModel& operator=(const SymbolicModel&)=delete;
  /* @endcond */

  /** 
   * @brief Construct SymbolicModel with the SymbolicSet representing the
   * state alphabet (pre and post) and input alphabet
   **/
  SymbolicModel(const SymbolicSet& pre,
                const SymbolicSet& input,
                const SymbolicSet& post) :
                m_pre(pre), m_input(input), m_post(post),
                m_z(new double[m_pre.get_dim()]()) {
    /* default value of the measurement error 
     * (heurisitc to prevent rounding errors)*/
    for(int i=0; i<m_pre.get_dim(); i++)
      m_z[i]=m_pre.get_eta()[i]/1e10;
  }

  template<class F1, class F2>
  BDD compute_gb(const Cudd& manager, F1& system_post, F2& radius_post, size_t& no_trans) {
    return compute_gb(manager, system_post, radius_post, [](const abs_type&) noexcept {return false;}, no_trans);
  }
  /** 
   * @brief computes the transition function
   *
   * @param [in]  manager     - Cudd manager
   *              
   * @param [in]  system_post - lambda expression as defined in  Abstraction::compute_gb
   *              
   * @param [in]  radius_post - lambda expression as defined in  Abstraction::compute_gb
   *              
   * @param [in]  avoid       - lambda expression as defined in  Abstraction::compute_gb
   *              
   * @param [out] no_trans    - number of transitions 
   *
   * @result              a BDD encoding the transition function as boolean function over
   *                      the BDD var IDs in m_pre, m_input, m_post
   **/
  template<class F1, class F2, class F3>
  BDD compute_gb(const Cudd& manager, F1& system_post, F2& radius_post, F3&& avoid, size_t& no_trans) {
    /* number of cells */
    abs_type N=m_pre.size(); 
    /* number of inputs */
    abs_type M=m_input.size();
    /* state space dimension */
    int dim=m_pre.get_dim();
    /* for display purpose */
    abs_type counter=0;
    /* variables for managing the post */
    std::vector<abs_type> lb(dim);  /* lower-left corner */
    std::vector<abs_type> ub(dim);  /* upper-right corner */
    /* radius of hyper interval containing the attainable set */
    state_type eta;
    state_type r;
    /* state and input variables */
    state_type x;
    input_type u;
    /* for out of bounds check */
    state_type lower_left;
    state_type upper_right;
    /* copy data from m_state_alphabet */
    for(int i=0; i<dim; i++) {
      eta[i]=m_pre.get_eta()[i];
      lower_left[i]=m_pre.get_lower_left()[i];
      upper_right[i]=m_pre.get_upper_right()[i];
    }
    /* the BDD to encode the transition function */
    BDD tf = manager.bddZero();
    /* is post of (i,j) out of domain ? */
    bool out_of_domain;
    /* loop over all cells */
    for(abs_type i=0; i<N; i++) {
      BDD bdd_i = m_pre.id_to_bdd(i);
      /* is i an element of the avoid symbols ? */
      if(avoid(i)) {
        continue;
      }
      /* loop over all inputs */
      for(abs_type j=0; j<M; j++) {
        BDD bdd_j = m_input.id_to_bdd(j);
        /* get center x of cell */
        m_pre.itox(i,x);
        /* cell radius (including measurement errors) */
        for(int k=0; k<dim; k++)
          r[k]=eta[k]/2.0+m_z[k];
        /* current input */
        m_input.itox(j,u);
        /* integrate system and radius growth bound */
        /* the result is stored in x and r */
        radius_post(r,x,u);
        system_post(x,u);
        /* determine the cells which intersect with the attainable set: 
         * discrete hyper interval of cell indices 
         * [lb[0]; ub[0]] x .... x [lb[dim-1]; ub[dim-1]]
         * covers attainable set */
        for(int k=0; k<dim; k++) {
          /* check for out of bounds */
          double left = x[k]-r[k]-m_z[k];
          double right = x[k]+r[k]+m_z[k];
          if(left <= lower_left[k]-eta[k]/2.0  || right >= upper_right[k]+eta[k]/2.0) {
            out_of_domain=true;
            break;
          } 
          /* integer coordinate of lower left corner of post */
          lb[k] = static_cast<abs_type>((left-lower_left[k]+eta[k]/2.0)/eta[k]);
          /* integer coordinate of upper right corner of post */
          ub[k] = static_cast<abs_type>((right-lower_left[k]+eta[k]/2.0)/eta[k]);
        }
        if(out_of_domain) {
          out_of_domain=false;
          continue;
        }
        /* compute BDD of post */
        BDD bdd_k = m_post.interval_to_bdd(manager,lb,ub);
        /* add to transition function */
        tf = tf | (bdd_i & bdd_j & bdd_k);
      }
      /* print progress */
      progress(i,N,counter);
    }

    /* count number of transitions */
    size_t nvars = m_pre.get_no_bdd_vars() +
                   m_input.get_no_bdd_vars() +
                   m_post.get_no_bdd_vars();
    no_trans=tf.CountMinterm(nvars);
    return tf;
  }
  
  /** @brief set the measurement error bound **/
  void set_measurement_error_bound(const state_type& error_bound) {
    for(int i=0; i<m_pre.get_dim(); i++) {
      m_z[i]=error_bound[i];
    }
  }

  /** @brief activate console output **/
  void verbose_on() {
    m_verbose=true;
  }

  /** @brief get measurement error bound **/
  std::vector<double> get_measruement_error_bound() const {
    std::vector<double> z;
    for(int i=0; i<m_pre.get_dim(); i++) {
      z.push_back(m_z[i]);
    }
    return z;
  }

  /** @brief deactivate console output **/
  void verbose_off() {
    m_verbose=false;
  }

  const SymbolicSet& get_sym_set_pre() const {
    return m_pre;
  }
  const SymbolicSet& get_sym_set_post() const {
    return m_post;
  }
  const SymbolicSet& get_sym_set_input() const {
    return m_input;
  }

};

} /* close namespace */
#endif /* SYMBOLICMODEL_HH_ */
