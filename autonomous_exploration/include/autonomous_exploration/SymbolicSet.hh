/*
 * SymbolicSet.hh
 *
 *  created: Jan 2016
 *   author: Matthias Rungger
 */

/** @file **/

#ifndef SYMBOLICSET_HH_
#define SYMBOLICSET_HH_


#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <numeric>

#include "UniformGrid.hh"
#include "IntegerInterval.hh"

/* cudd library */
#include "cuddObj.hh"

namespace scots {

/** 
 *  @class SymbolicSet 
 *
 *  @brief The UniformGrid class with the capability to represent sets of grid
 *  points as BDDs. See UniformGrid for details.
 *
 **/
class SymbolicSet : public UniformGrid {
private:
  /* a vector of IntegerIntervals - one for each dimension */
  std::vector<IntegerInterval<abs_type>> m_bdd_interval;
  /* a vector containing the slugs variable names */
  std::vector<std::string> m_slugs_var_names;
public:
  /** @brief construct SymbolicSet with a Cudd manager **/
  SymbolicSet() : UniformGrid(), m_bdd_interval{}, m_slugs_var_names{} { }

  /** @brief create a SymbolicSet from other by projecting it onto the dimension in dim **/
  SymbolicSet(const SymbolicSet& other, std::vector<int> dim) :
              UniformGrid(other,dim), m_bdd_interval{}, m_slugs_var_names{} {
    for(const auto& i : dim) {
      m_bdd_interval.push_back(other.m_bdd_interval[i]);
      if(other.m_slugs_var_names.size()>static_cast<size_t>(i))
        m_slugs_var_names.push_back(other.m_slugs_var_names[i]);
    }
  }
  
  /** @brief create a SymbolicSet and initialize m_bdd_interval with intervals **/
  SymbolicSet(const UniformGrid& grid,
              const std::vector<IntegerInterval<abs_type>>& intervals) :
              UniformGrid(grid), m_bdd_interval(intervals), m_slugs_var_names{} { 
  }

  /**
   * @brief provide BDD variable manager and  UniformGrid parameters 
   * 
   * @param manager  - BDD variable manager 
   * @param dim      - dimension of the real space
   * @param lb       - lower-left corner of the hyper-interval confining the uniform grid
   * @param ub       - upper-right corner of the hyper-interval confining the uniform grid
   * @param eta      - grid point distances
   * OPTIONAL
   * @param names    - vector of size dim with names of integer variables used to interface with slugs
   **/
  template<class grid_point_t>
  SymbolicSet(const Cudd& manager,
              const int dim,
              const grid_point_t& lb,
              const grid_point_t& ub,
              const grid_point_t& eta,
              const std::vector<std::string> names = {}) :
              UniformGrid(dim,lb,ub,eta), m_bdd_interval{}, m_slugs_var_names(names) {
    for(int i=0; i<m_dim; i++) 
      m_bdd_interval.emplace_back(manager,abs_type{0},m_no_grid_points[i]-1);
  }

  /**
   * @brief provide BDD variable manager and UniformGrid 
   * 
   * @param manager  - BDD variable manager 
   * @param grid     - UnfiormGrid
   * OPTIONAL
   * @param names    - vector of size dim with names of integer BDD variables used to interface with slugs
   **/
  SymbolicSet(const Cudd& manager,
              const UniformGrid& grid,
              const std::vector<std::string> names = {}) :
              UniformGrid(grid), m_bdd_interval{}, m_slugs_var_names(names) {
    for(int i=0; i<m_dim; i++) 
      m_bdd_interval.emplace_back(manager,abs_type{0},m_no_grid_points[i]-1);
  }
  /**
   * @brief construct product of two SymbolicSets
   * 
   * The instantiated SymbolicSet represents the Cartesian product of the
   * SymbolicSet set1 and the SymbolicSet set2
   * 
   * @param set1  - SymbolicSet
   * @param set2  - SymbolicSet
   **/
  SymbolicSet(const SymbolicSet& set1, const SymbolicSet& set2) {
    m_dim = set1.m_dim + set2.m_dim;
    m_eta.reset(new double[m_dim]);
    m_first.reset(new double[m_dim]);
    m_no_grid_points.reset(new abs_type[m_dim]);
    m_NN.reset(new abs_type[m_dim]);
    for(int i=0; i<set1.m_dim; i++) {
      m_eta[i] = set1.m_eta[i];
      m_first[i]  = set1.m_first[i];
      m_no_grid_points[i]  = set1.m_no_grid_points[i];
      m_bdd_interval.push_back(set1.m_bdd_interval[i]);
      }
    for(int j=set1.m_dim, i=0; i<set2.m_dim; i++) {
      m_eta[j+i] = set2.m_eta[i];
      m_first[j+i]  = set2.m_first[i];
      m_no_grid_points[j+i]  = set2.m_no_grid_points[i];
      m_bdd_interval.push_back(set2.m_bdd_interval[i]);
    }
    calc_nn();
    /* copy slugs names */
    if(set1.m_slugs_var_names.size() || set2.m_slugs_var_names.size()) {
      if(set1.m_slugs_var_names.size())
        m_slugs_var_names=set1.m_slugs_var_names;
      else
        m_slugs_var_names.resize(set1.m_dim, std::string{'d'});
      if(set2.m_slugs_var_names.size())
        m_slugs_var_names.insert(m_slugs_var_names.end(), set2.m_slugs_var_names.begin(), set2.m_slugs_var_names.end());
      else
        m_slugs_var_names.resize(set1.m_dim+set2.m_dim, std::string{'d'});
    }

  }

  /** @brief print information about the symbolic set **/
  void print_info(int verbose=0) const {
    UniformGrid::print_info();
    std::cout << "Number of BDD variables per dimension ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_bdd_interval[i].get_no_bdd_vars() << " ";
    }
    if(verbose) {
      std::vector<int> no_bdd_var{0};
      std::cout << "\n";
      for(int i=0; i<m_dim; i++) {
        std::cout << "Dim " << i+1 << ": ";
        m_bdd_interval[i].print_bdd_IDs();
        no_bdd_var.push_back(no_bdd_var.back()+m_bdd_interval[i].get_no_bdd_vars());
      }
      if(m_slugs_var_names.size()) {
        auto slugs_names = get_slugs_var_names();
        std::cout << "Bdd variable names (to interface with slugs)\n";
        for(int i=0; i<m_dim; i++) {
          std::cout << "Integer var in dim " << i+1 << ": ";
          for(int j=no_bdd_var[i]; j<no_bdd_var[i+1]; j++) {
              std::cout << slugs_names[j] << " ";
          }
          std::cout << std::endl;
        }
      }
    }
    std::cout << "\n";
  }

  /** @brief function to obtain a BDD representation of the grid point id **/
  BDD id_to_bdd(abs_type id) const {
    abs_type num;
    int k=m_dim-1;
    /* k= m_dim -1 */
    num=id/m_NN[k];
    id=id%m_NN[k];
    BDD bdd = m_bdd_interval[k].int_to_bdd(num);
    for(k=m_dim-2; k >= 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      bdd = bdd & m_bdd_interval[k].int_to_bdd(num);
    }
    return bdd;
  }

  /** @brief get a BDD representation of the grid points  whose IDs are 
   *  an element of the integer hyper-interval [lb; ub] **/
  BDD interval_to_bdd(const Cudd& manager,
                      const std::vector<abs_type>& lb,   
                      const std::vector<abs_type>& ub) const {
    BDD bdd = m_bdd_interval[0].interval_to_bdd(manager,lb[0],ub[0]);
    for(int i=1; i<m_dim; i++) 
      bdd = bdd & m_bdd_interval[i].interval_to_bdd(manager,lb[i],ub[i]);
    return bdd;
  }

  /**
   * @brief obtain a BDD representation of the grid points whose grid point IDs
   * evaluate to true in the lambda expression
   * \verbatim [](const abs_type& i) -> bool \endverbatim
   **/
  template<class F>
  BDD ap_to_bdd(const Cudd& manager, const F& atomic_prop) const {
    BDD result = manager.bddZero();; 
    for(abs_type i=0; i<size(); i++) {
      if(atomic_prop(i)) 
        result = result | id_to_bdd(i);
    }
    return result;
  }


  /** @brief get a vector of grid points that are encoded in the BDD
   *
   *  The return vector is of size (number of grid points) x n where n is the  dimension.\n 
   *  The grid points are stacked on top of each other, i.e., the first n
   *  entries of the return vector represent the first grid point.
   **/
  std::vector<double> bdd_to_grid_points(const Cudd& manager, BDD bdd) {
    if((!get_no_bdd_vars()) || bdd==manager.bddZero())
      return {};
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();
    /* get variable ids */
    auto var_id = get_bdd_var_ids();
    /* find the variables in the support of the BDD but outside the SymbolicSet */
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    /* init the vector of grid points to be returned */
    abs_type no_gp = get_size(manager,bdd);
    std::vector<double> gp(no_gp*m_dim);
    for(abs_type i=0; i<no_gp; i++) {
      for(int j=0; j<m_dim; j++)
        gp[i*m_dim+j]=m_first[j];
    }
    /* set up iteration to iterate over BDD cubes */
    DdManager* dd = manager.getManager();
    int *cube;
    CUDD_VALUE_TYPE value;
    DdGen *gen;
    abs_type counter=0;
    /* iterate over BDD cubes */
    Cudd_ForeachCube(dd,bdd.getNode(),gen,cube,value) {
      abs_type offset=1;
      for(int i=0; i<m_dim; i++) {
        unsigned int no_vars = m_bdd_interval[i].get_no_bdd_vars();
        for (unsigned int j=0; j<no_vars; j++) {
          unsigned int id = m_bdd_interval[i].get_bdd_var_ids()[j];
          if(cube[id]==1) {
            for(abs_type k=0; k<offset; k++) {
              gp[(counter+k)*m_dim+i]+=(abs_type{1}<<(no_vars-1-j))*m_eta[i];
            }
          }
          /* take care of don't care */
          if(cube[id]==2) {
            for(abs_type k=0; k<offset; k++) {
              for(int l=0; l<=i; l++) {
                gp[(counter+k+offset)*m_dim+l]=gp[(counter+k)*m_dim+l];
              }
            }
            for(abs_type k=0; k<offset; k++) {
              gp[(counter+k+offset)*m_dim+i]+=(abs_type{1}<<(no_vars-1-j))*m_eta[i];
            }
            offset=(offset<<1);
          }
        }
      }
      counter+=offset;
    }
    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);
    return gp;
  }

  /** @brief projection of the set of grid points onto the dimension in dim+1,
   * e.g., to project onto the 1st and 2nd coordinate, set dim = {0,1}  **/
  std::vector<double> projection(const Cudd& manager, const BDD& bdd, std::vector<int> dim) const {
    if(!dim.size())
      return {};
    return SymbolicSet(*this,dim).bdd_to_grid_points(manager,bdd);
  }

  /**
   * @brief restriction of the set of grid points (represented as the BDD bdd) to x
   *
   * Consider the set \f$S\f$ represented by the BDD bdd, being a subset of the
   * n-dimensional hyper-interval 
   * \f$
   *  S\subseteq ( [lb_0,ub_0] \times \cdots [lb_{n-1},ub_{n-1}] ) \cap \eta  \mathbb Z^n
   * \f$.
   * Let \f$x=(x_{j_0},\ldots, x_{j_{m-1}})\f$ with \f$m<n\f$ and \f$p=n-m\f$. In
   * the default setting, the indices \f$j_i\f$ are set to \f$j_i=i\f$. Then the
   * function returns the grid points in the p-dimensional hyper-interval\n\n
   * \f$
   *  S(x):= \{ y\mid (x_{j_0},\ldots, x_{j_m},y_{0},\ldots,y_{p})\in S \}.
   * \f$
   * \n\n
   * Note that \f$S(x) \subseteq [lb_{m},ub_{m}]\times\cdots\times
   * [lb_{n-1},ub_{n-1}] ) \cap \eta  \mathbb Z^n\f$. Optionally, provide the
   * m-dimensional vector domain defining the indices
   * \f$j_0,\ldots,j_{m-1}\f$.
   * For example, if \f$n=4\f$, \f$m=2\f$ and \f$j_0=1,j_1=3\f$, then the
   * function returns the grid points in \n\n
   * \f$
   *  S(x):= \{ (y_0,y_1)\mid (y_0,x_1,y_1,x_3)\in S \}.
   * \f$
   * 
   * @param bdd      the BDD that is used to map 
   * @param x        the element to which the relation is restricted \n
   *                 if domain is not provided, then grid_point_t needs to have grid_point_t::size() method
   * @param domain   Optinally, provide the indices over which the elements of
   *                   x are interpreted
   * @result          vector of grid points  \f$ S(x) \f$ 
   **/
  template<class grid_point_t>
  std::vector<double> restriction(const Cudd& manager,
                                  const BDD& bdd, 
                                  const grid_point_t& x,  
                                  std::vector<int> domain = {}) const {
    /* compute indices in domain/codomain */
    std::vector<int> codomain {};
    /* fill in default values */
    if(domain.size()==0) {
      domain.resize(x.size());
      std::iota(std::begin(domain),std::end(domain),0);
      codomain.resize(m_dim-x.size());
      std::iota(std::begin(codomain),std::end(codomain),x.size());
    } else {
      for(int i=0; i<m_dim; i++) {
        if(std::find(std::begin(domain), std::end(domain), i) == std::end(domain))
          codomain.push_back(i);
      }
    }
    /* create SymbolicSet of the domain and codmain */
    SymbolicSet set_dom(*this,domain);
    SymbolicSet set_codom(*this,codomain);
    /* extract bdd with restriction */
    abs_type id = set_dom.xtoi(x);
    BDD restricted = bdd & set_dom.id_to_bdd(id);
    restricted = restricted.ExistAbstract(set_dom.get_cube(manager));
    /* map restricted BDD to grid points */
    return set_codom.bdd_to_grid_points(manager,restricted);
  }

  /** @brief get number of BDD variables used to represent the SymbolicSet **/
  abs_type get_no_bdd_vars() const {
    abs_type num = 0;
    for(const auto& interval : m_bdd_interval) 
      num+=interval.get_no_bdd_vars();
    return num;
  }

  /** @brief get number of BDD variables used to represent the SymbolicSet **/
  std::vector<BDD> get_bdd_vars() const {
    std::vector<BDD> vars {};
    for(const auto& interval : m_bdd_interval) {
      std::vector<BDD> var = interval.get_bdd_vars();
      for(const auto& p : var) {
        vars.push_back(p);
      }
    }
    return vars;
  }

  /** @brief get number of BDD variables used to represent the SymbolicSet **/
  std::vector<unsigned int> get_bdd_var_ids() const {
    std::vector<unsigned int> var_id {};
    for(const auto& interval : m_bdd_interval) {
      for(const auto& id : interval.get_bdd_var_ids()) 
        var_id.push_back(id);
    }
    return var_id;
  }

  /** @brief get a vector of IDs that corresponding to the grid points encoded by the BDD **/
  std::vector<abs_type> bdd_to_id(const Cudd& manager, BDD bdd) {
    if((!get_no_bdd_vars()) || bdd==manager.bddZero())
      return {};
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();
    /* get variable ids */
    auto var_id = get_bdd_var_ids();
    /* find the variables in the support of the BDD but outside the SymbolicSet */
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
      if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
        out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    /* init the vector of grid points to be returned */
    abs_type no_id = get_size(manager,bdd);
    std::vector<abs_type> IDs(no_id,0);

    /* set up iteration to iterate over BDD cubes */
    DdManager* dd = manager.getManager();
    int *cube;
    CUDD_VALUE_TYPE value;
    DdGen *gen;
    abs_type counter=0;
    /* iterate over BDD cubes */
    Cudd_ForeachCube(dd,bdd.getNode(),gen,cube,value) {
      abs_type offset=1;
      for(int i=0; i<m_dim; i++) {
        unsigned int no_vars = m_bdd_interval[i].get_no_bdd_vars();
        for (unsigned int j=0; j<no_vars; j++) {
          unsigned int id = m_bdd_interval[i].get_bdd_var_ids()[j];
          if(cube[id]==1) {
            for(abs_type k=0; k<offset; k++) {
              IDs[counter+k]+=(abs_type{1}<<(no_vars-1-j))*m_NN[i];
            }
          }
          /* take care of don't care */
          if(cube[id]==2) {
            for(abs_type k=0; k<offset; k++) {
              for(int l=0; l<=i; l++) {
                IDs[counter+k+offset]=IDs[counter+k];
              }
            }
            for(abs_type k=0; k<offset; k++) {
              IDs[counter+k+offset]+=(abs_type{1}<<(no_vars-1-j))*m_NN[i];
            }
            offset=(offset<<1);
          }
        }
      }
      counter+=offset;
    }

    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);
    return IDs;
  }


  /** @brief get number grid points represented by the BDD  **/
  abs_type get_size(const Cudd& manager, BDD bdd) const {
    /* find the variables in the support of the BDD but outside the SymbolicSet */
    auto var_id = get_bdd_var_ids();
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    return static_cast<abs_type>(bdd.CountMinterm(get_no_bdd_vars()));
  } 

  /** @brief limit the support of the bdd to the variables in the SymbolicSet  **/
  void clean(const Cudd& manager, BDD &bdd) const {
    /* find the variables in the support of the BDD but outside the SymbolicSet */
    auto var_id = get_bdd_var_ids();
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
  } 

  /** @brief get IntegerInterval  **/
  std::vector<IntegerInterval<abs_type>> get_bdd_intervals() const {
    return m_bdd_interval;
  }
  /** @brief get cube BDD with the BDD variables of the SymbolicSet **/
  BDD get_cube(const Cudd& manager) const {
    return manager.computeCube(get_bdd_vars());
  }

  /** @brief set bdd variable names (used to interface with slugs) **/
  void set_slugs_var_names(const std::vector<std::string>& var_names) { 
    m_slugs_var_names = var_names;
  }

  /** @brief get bdd variable names (used to interface with slugs) **/
  std::vector<std::string> get_slugs_var_names() const { 
    std::vector<std::string> var_names {};
    if(m_slugs_var_names.size()) {
      for(size_t i=0; i<m_bdd_interval.size(); i++) {
        for(const auto& bdd_names : m_bdd_interval[i].get_slugs_var_names()) {
          if(m_slugs_var_names[i].back()=='\'') {
            std::string tmp = m_slugs_var_names[i];
            /* remove prime from name  */
            tmp.pop_back();
            var_names.push_back(tmp+bdd_names);
            /* add prime at the end of name  */
            var_names.back().push_back('\'');
          }
          else 
            var_names.push_back(m_slugs_var_names[i]+bdd_names);
        }
      }
    } 
    return var_names;
  }

}; /* close class def */
} /* close namespace */
#endif /* SYMBOLICSET_HH_ */
