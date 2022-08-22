/*
 * GameSolver.hh
 *
 *  created: Feb 2016
 *   author: Matthias Rungger
 * 
 */

/** @file **/

#ifndef GAMESOLVER_HH_
#define GAMESOLVER_HH_

#include <iostream>
#include <climits>
#include <stdexcept>
#include <queue>
#include <memory>
#include <utility>

#include "UniformGrid.hh"
#include "TransitionFunction.hh"
#include "WinningDomain.hh"


/** @namespace scots **/ 
namespace scots {

/** @cond **/
/* default parameters for the solve_reachability_game */
namespace params {
  auto avoid = [](const abs_type&) noexcept {return false;};
  static std::vector<double> value {};
}
/** @endcond **/

/**
 * @brief solve reachability game according to Algorithm 2 in  <a href="./../../manual/manual.pdf">manual</a>
 * 
 * @param[in] trans_function - TransitionFunction of the symbolic model
 * @param[in] target - lambda expression of the form
 *                      \verbatim [] (const abs_type &i) -> bool \endverbatim 
 *                      returns true if state i is in target set and false otherwise
 *                       
 * @param[in] avoid  - OPTIONALLY provide lambda expression of the form
 *                      \verbatim [] (const abs_type &i) -> bool \endverbatim
 *                      returns true if state i is in avoid set and false otherwise
 * 
 * @param[out] value - OPTIONALLY provide std::vector<double> value to obtain the value function 
 *
 * @return -  WinningDomain that contains the set of winning states and valid inputs 
 **/
template<class F1, class F2=decltype(params::avoid)>
WinningDomain solve_reachability_game(const TransitionFunction& trans_function,
                                      F1& target, 
                                      F2& avoid = params::avoid,
                                      std::vector<double> & value = params::value ) {
  /* size of state alphabet */
  abs_type N=trans_function.m_no_states;
  /* size of input alphabet */
  abs_type M=trans_function.m_no_inputs;

  /* used to encode that a state is not in the winning domain */
  abs_type loosing = std::numeric_limits<abs_type>::max();
  if(M > loosing-1) {
    throw std::runtime_error("scots::solve_reachability_game: Number of inputs exceeds maximum supported value");
  }
  /* win_domain[i] = j
   * contains the input j associated with state i
   *
   * j = loosing if the target is not reachable from i 
   *
   * initialize all states to loosing (win_domain[i]=loosing) */
  std::vector<abs_type> win_domain(N,loosing); 
  /* initialize value */
  value.resize(N,std::numeric_limits<double>::infinity());
  /* keep track of the number of processed post */
  std::unique_ptr<abs_type[]> K(new abs_type[N*M]);
  /* keep track of the values (corresponds to M in Alg.2)*/
  std::unique_ptr<double[]>  edge_val(new double[N*M]);

  /* init fifo */
  std::queue<abs_type> fifo;
  for(abs_type i=0; i<N; i++) {
    if(target(i) && !avoid(i)) {
      win_domain[i]=loosing;
      /* value is zero */
      value[i]=0;
      /* states in the target are added to the fifo */
      fifo.push(i);
    }
    for(abs_type j=0; j<M; j++) {
      edge_val[i*M+j]=0;
      K[i*M+j]=trans_function.m_no_post[i*M+j];
    }
  }

  /* main loop */
  while(!fifo.empty()) {
    /* get state to be processed */
    abs_type q=fifo.front();
    fifo.pop();
    /* loop over each input */
    for(abs_type j=0; j<M; j++) {
      /* loop over pre's associated with this input */
      for(abs_ptr_type v=0; v<trans_function.m_no_pre[q*M+j]; v++) {
        abs_type i=trans_function.m_pre[trans_function.m_pre_ptr[q*M+j]+v];
        if(avoid(i))
          continue;
        /* (i,j,q) is a transition */
        /* update the number of processed posts */
        K[i*M+j]--;
        /* update the max value of processed posts */
        edge_val[i*M+j]=(edge_val[i*M+j]>=1+value[q] ? edge_val[i*M+j] : 1+value[q]);
        /* check if for node i and input j all posts are processed */
        if(!K[i*M+j] && value[i]>edge_val[i*M+j]) {
          fifo.push(i);
          value[i]=edge_val[i*M+j]; 
          win_domain[i]=j;
        }
      }  /* end loop over all pres of state i under input j */
    }  /* end loop over all input j */
  }  /* fifo is empty */

  /* if the default value function was used, free the memory of the static object*/
  if(value == scots::params::value){
      value.clear();
      value.shrink_to_fit();
  }

  return WinningDomain(N,M,std::move(win_domain),std::vector<bool>{},loosing);
}

/**
 * @brief solve invariance game according to Algorithm 1 in  <a href="./../../manual/manual.pdf">manual</a>
 * 
 * @param[in] trans_function - TransitionFunction of the symbolic model
 * @param[in] safe - lambda expression of the form
 *                    \verbatim [] (const abs_type &i) -> bool \endverbatim 
 *                   returns true if state i is in safe set and false otherwise
 * @return -  WinningDomain that contains the set of winning states and valid inputs 
 **/
template<class F>
WinningDomain solve_invariance_game(const TransitionFunction& trans_function, F& safe) {
  /* size of state alphabet */
  abs_type N=trans_function.m_no_states;
  /* size of input alphabet */
  abs_type M=trans_function.m_no_inputs;
  /* used to encode that a state is not in the winning domain */
  abs_type loosing = std::numeric_limits<abs_type>::max();

  /* valid_inputs
   * boolean array of size N*M
   * valid_inputs[i*M+j]=true iff input j
   * is a valid input at state i */
  std::vector<bool> valid_inputs(N*M,false); 
  /* no_input: keep track of the number of valid inputs.
   * If no_val_in[i]==0, then state i is not winning and 
   * no_val_in[i] is set to loosing (see also WinningDomain) */
  std::vector<abs_type> no_val_in(N,0);
  /* keep track if an unsafe state was already added to the fifo */
  std::vector<bool> added(N,false);

  /* initialization */
  std::queue<abs_type> fifo;
  for(abs_type i=0; i<N; i++) {
    if(safe(i)) {
      for(abs_type j=0; j<M; j++) {
        if(trans_function.m_no_post[i*M+j]) {
          valid_inputs[i*M+j]=true;
          no_val_in[i]++;
        }
      }
    }
    if(!no_val_in[i]) {
      fifo.push(i);
      added[i]=true;
      /* mark no_val_in[i]=loosing to indicate that state i ist not winning */
      no_val_in[i]=loosing;
    }
  }

  while(!fifo.empty()) {
    abs_type k=fifo.front();
    fifo.pop();
    /* loop over all inputs */
    for(abs_type j=0; j<M; j++) {
      /* loop over all pre states of (k,j) */
      for(abs_ptr_type p=0; p<trans_function.m_no_pre[k*M+j]; p++) {
        /* (i,j,k) is a transition */
        abs_type i=trans_function.m_pre[trans_function.m_pre_ptr[k*M+j]+p];
        /* check if input j at state i is considered safe */
        if(valid_inputs[i*M+j]) {
          /* set source states i with label j as unsafe pair */
          valid_inputs[i*M+j]=false;
          no_val_in[i]--;
        }
        /* add to unsafe set if state i has no more valid inputs */
        if(!no_val_in[i] && !added[i]) {
          fifo.push(i);
          added[i]=true;
          /* mark no_val_in[i]=loosing to indicate that state i ist not winning */
          no_val_in[i]=loosing;
        }
      }
    }
  }
  return WinningDomain(N,M,std::move(no_val_in),std::move(valid_inputs),loosing);
}

} /* end of namespace scots */
#endif /* GAMESOLVER_HH_ */
