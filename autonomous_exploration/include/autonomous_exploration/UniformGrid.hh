/*
 * UniformGrid.hh
 *
 *     created: Dec 2016
 *      author: Matthias Rungger
 *              Frederik Kunik
 */

/** @file **/

#ifndef UNIFORMGRID_HH_
#define UNIFORMGRID_HH_

#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <exception>
#include <algorithm>
#include <climits>
#include <memory>

/** @namespace scots **/ 
namespace scots {

/**
 * @brief abs_type defines type of abstract states (default = uint32_t) 
 *
 * It is required to be an integer type. It determines implicitly an upper
 * bound on the number of abstract states (default = 2^32-1). 
 **/
#ifndef SCOTS_BDD
using abs_type=std::uint32_t;
#else 
using abs_type=std::uint64_t;
#endif 

/**
 * @class UniformGrid 
 *
 * @brief Holds the information of a uniform grid confined by the hyper-interval
 * [lb, ub]
 *
 *
 * Grid point alignment: 
 * - the grid points are distributed uniformly in each dimension 
 * - the origin is a grid point (not necessarily contained in the set) 
 * - the distance of the grid points in each dimension i is defined by eta[i] 
 *
 * Each grid point is associated with an ID of type abs_type (default
 * abs_type=uint32_t). The abs_type need to be an integral type and implicitly determines an upper bound on the number of
 * grid points that are representable by the UniformGrid.
 *
 *
 * A UniformGrid is often used to encode the state alphabet (or real quantizer
 * symbols) of an abstraction. In that context, we interpret the grid points as
 * centers of cells and the grid point IDs as cell IDs. The radius of the cells
 * is given by \f$\eta/2\f$. 
 *
 * The member functions itox and xtoi are used to map between the IDs and the
 * grid points/center of the cells.
 *
 * See 
 * - the manual in <a href="./../../manual/manual.pdf">manual</a>
 * - http://arxiv.org/abs/1503.03715 for theoretical background 
 **/
class UniformGrid {
protected:
  /** @brief dimension of the Eucleadian space **/
  int m_dim;                
  /** @brief m_dim-dimensional vector containing the grid node distances **/
  std::unique_ptr<double[]> m_eta;                      
  /** @brief m_dim-dimensional vector containing the real values of the first grid point **/
  std::unique_ptr<double[]> m_first;        
  /** @brief scots::abs_type array[m_dim] containing the number of grid points in each dimension **/
  std::unique_ptr<abs_type[]> m_no_grid_points;        
  /** @brief array recursively defined by: m_NN[0]=1; m_NN[i]=m_NN[i-1}*no_grid_points[i-1]; **/
  std::unique_ptr<abs_type[]> m_NN;                       

public:
  /* @cond  EXCLUDE from doxygen */
  /* default constructor */
  UniformGrid() : m_dim(0),
                  m_eta(nullptr), 
                  m_first(nullptr),
                  m_no_grid_points(nullptr),
                  m_NN(nullptr) { }
  /* destructor */
  virtual
  ~UniformGrid() = default;
  /* copy constructor */
  UniformGrid(const UniformGrid& other) : UniformGrid() {
    *this=other;
  }
  /* move constructor */
  UniformGrid(UniformGrid&& other) : UniformGrid() {
    *this=std::move(other);
  }
  /* copy assignment operator */
  UniformGrid& operator=(const UniformGrid &other) {
    if(this==&other)
      return *this;
    m_dim=other.m_dim;
    if(m_dim != 0) {
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);
      for(int i=0; i<m_dim; i++) {
        m_eta[i] = other.m_eta[i];
        m_first[i]  = other.m_first[i];
        m_no_grid_points[i]  = other.m_no_grid_points[i];
        m_NN[i]  = other.m_NN[i];
      }
    } 
    return *this;
  }
  /* create UniformGrid from other by projection on the dimension specified in dim */
  UniformGrid(const UniformGrid &other, const std::vector<int>& dim) : UniformGrid() {
    m_dim=dim.size();
    if(m_dim != 0) {
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);
      for(int i=0; i<m_dim; i++) {
        m_eta[i] = other.m_eta[dim[i]];
        m_first[i]  = other.m_first[dim[i]];
        m_no_grid_points[i]  = other.m_no_grid_points[dim[i]];
      }
      calc_nn();
    } 
  }
  /* move assignment operator */
  UniformGrid& operator=(UniformGrid&& other) {
    m_dim=std::move(other.m_dim);
    m_eta=std::move(other.m_eta);
    m_first=std::move(other.m_first);
    m_no_grid_points=std::move(other.m_no_grid_points);
    m_NN=std::move(other.m_NN);
    return *this;
  } 
  /* @endcond */

  /**
   * @brief provide uniform grid parameters and domain defining hyper-interval 
   * 
   * @param dim   - dimension of the real space
   * @param lb    - lower-left corner of the hyper-interval confining the uniform grid
   * @param ub    - upper-right corner of the hyper-interval confining the uniform grid
   * @param eta   - grid point distances
   **/
  template<class grid_point_t>
  UniformGrid(const int dim,
              const grid_point_t& lb,
              const grid_point_t& ub,
              const grid_point_t& eta) : UniformGrid() {
    m_dim = dim;
    if(m_dim != 0) {
      /* check inut arguments */
      for(int index=0; index<dim; index++) {
        if(eta[index] <= 0) 
          throw std::runtime_error("\nscots::UniformGrid: eta must have positive entries.");
        if(lb[index] > ub[index]) 
          throw std::runtime_error("\nscots::UniformGrid: lower-left bound must be less than or equal to upper-right bound.");
      }
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);

      /* determine number grid points in each dimension */
      std::size_t no_l, no_u;
      int sign_l, sign_u;
      for(int index=0; index<m_dim; index++) {
        m_eta[index] = eta[index];
        /* rounding heuristic */
        double delta = m_eta[index]/1E10;
        try {
          /* get sign */
          sign_l = (lb[index] > 0) ? 1 : ((lb[index] < 0) ? -1 : 0);
          /* compute number of grid points from zero to lower bound */
          no_l=std::llround(std::abs(lb[index])/eta[index]+sign_l*(0.5-delta));
        } catch (...) {
          std::ostringstream os;
          os << "\nscots::UniformGrid: something wrong in the division of " << lb[index] << " by " << eta[index] ;
          throw std::runtime_error(os.str().c_str());
        }
        try {
          /* get sign */
          sign_u = (ub[index] > 0) ? 1 : ((ub[index] < 0) ? -1 : 0);
          /* compute number of grid points from zero to upper bound */
          no_u=std::llround(std::abs(ub[index])/eta[index]-sign_u*(0.5-delta));
        } catch (...) {
          std::ostringstream os;
          os << "\nscots::UniformGrid: something wrong in the division of " << ub[index] << " by " << eta[index] ;
          throw std::runtime_error(os.str().c_str());
        }
        /* check if number of grid points in dimension index does not exceed max representable by abs_type  */
        if((sign_u*no_u-sign_l*no_l+1) > std::numeric_limits<abs_type>::max()) {
          std::ostringstream os;
          throw std::runtime_error("\nscots::UniformGrid: number of grid points exceeds maximum value of abs_type.");
        }
        m_no_grid_points[index] = sign_u*no_u-sign_l*no_l+1;
        /* first grid point coordinates */
        m_first[index]= (double)sign_l*(double)no_l*eta[index];
      }
      calc_nn();
    } else {
      throw std::runtime_error("\nscots::UniformGrid: grid dimension has to be greater than zero (using non-default constructor)");
    }
  }

  /** @brief compute the index associated with a grid point **/
  template<class grid_point_t>
  abs_type xtoi(const grid_point_t& x) const {
    abs_type id = 0;
    double d_id;
    double eta_h;

    for(int k=0; k<m_dim; k++) {
      d_id = x[k]-m_first[k];
      eta_h = m_eta[k]/2.0;

      if ( d_id <= -eta_h || d_id >= m_no_grid_points[k]*m_eta[k]+eta_h ) {
        std::ostringstream os;
        os << "\nscots::UniformGrid: state ";
        for(int i=0; i<m_dim; i++) {
          os << x[i] << " ";
        }
        os << "is outside uniform grid.";
        throw std::runtime_error(os.str().c_str());
      }
      id += static_cast<abs_type>((d_id+eta_h )/m_eta[k])*m_NN[k];
    }
    return id;
  }

  /** @brief compute the grid point associated with a index **/
  template<class grid_point_t>
  void itox(abs_type id, grid_point_t& x) const {
    /* map index id to grid point */
    abs_type num;
    for(int k = m_dim-1; k > 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      x[k]=m_first[k]+num*m_eta[k];
    }
    num=id;
    x[0]=m_first[0]+num*m_eta[0];
  }

  /** @brief compute the grid point associated with a index **/
  void itox(abs_type id, std::vector<double>& x) const {
    x.resize(m_dim);
    /* map index id to grid point */
    abs_type num;
    for(int k = m_dim-1; k > 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      x[k]=m_first[k]+num*m_eta[k];
    }
    num=id;
    x[0]=m_first[0]+num*m_eta[0];
  }

  /** @brief do a index to state conversion for vectors **/
  template<class grid_point_t>
  std::vector<grid_point_t> ItoX(std::vector<abs_type>& Ivector) const{

        std::vector<grid_point_t> Xvector;
        grid_point_t x;

        for(abs_type k=0; k<Ivector.size(); k++) {
          itox<grid_point_t>(Ivector[k],x);
          Xvector.push_back(x);
        }
        return Xvector;
  }

  /** @brief do a state to index conversion for vectors **/
  template<class grid_point_t>
  std::vector<abs_type> XtoI(std::vector<grid_point_t>& Xvector) const{

        std::vector<abs_type> Ivector;

        for(abs_type k=0; k<Xvector.size(); k++) {
          Ivector.push_back(xtoi<grid_point_t>(Xvector[k]));
        }
        return Ivector;
  }

  /** @brief creates console output with grid information **/
  void print_info() const {
    std::cout << "Distance of grid points (eta): ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_eta[i] << " ";
    }
    std::cout << "\nLower-left grid point: ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_first[i] << " ";
    }
    std::cout << "\nUpper-right grid point: ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_first[i]+m_eta[i]*(m_no_grid_points[i]-1) << " ";
    }
    std::cout << "\nNumber of grid points in each dimension: ";
    for(int i=0; i<m_dim; i++) {
        std::cout << m_no_grid_points[i] << " ";
    }
    std::cout << "\nNumber of grid points: "<< total_no_grid_points() << std::endl;
  }

  /** @name get functions **/
  //@{
  int get_dim() const {
    return m_dim;
  }
  /* total number of grid points */
  abs_type size() const {
    return total_no_grid_points();
  }
  std::vector<double> get_eta() const {
    std::vector<double> eta;
    for(int i=0; i<m_dim; i++) {
      eta.push_back(m_eta[i]);
    }
    return eta;
  }
  std::vector<double> get_lower_left() const {
    std::vector<double> lower_left;
    for(int i=0; i<m_dim; i++) {
      lower_left.push_back(m_first[i]);
    }
    return lower_left;
  }
  std::vector<double> get_upper_right() const {
    std::vector<double> upper_right;
    for(int i=0; i<m_dim; i++) {
      upper_right.push_back(m_first[i]+m_eta[i]*(m_no_grid_points[i]-1));
    }
    return upper_right;
  }
  std::vector<abs_type> get_no_gp_per_dim() const {
    std::vector<abs_type> no_grid_points;
    for(int i=0; i<m_dim; i++) {
      no_grid_points.push_back(m_no_grid_points[i]);
    }
    return no_grid_points;
  }
  std::vector<abs_type> get_nn() const {
    std::vector<abs_type> NN;
    for(int i=0; i<m_dim; i++) {
      NN.push_back(m_NN[i]);
    }
    return NN;
  }
  //@}

protected:
  void calc_nn() {
    /* compute m_NN */
    abs_type max = std::numeric_limits<abs_type>::max();
    abs_type total=1;
    for(int i=0; i<m_dim; i++) {
      m_NN[i] = total;
      /* check overflow */
      if(total > (max/m_no_grid_points[i])) {
        throw std::runtime_error("\nscots::UniformGrid: number of grid points exceeds maximum value of abs_type (defined in UniformGrid.hh).");
      }
      total *= m_no_grid_points[i];
    }
  }

private:
  /** @brief helper function to calculate the overall number of grid points **/
  abs_type total_no_grid_points() const {
    abs_type total=1;
    for(int i=0; i<m_dim; i++) {
      total *= m_no_grid_points[i];
    }
    return total;
  }
};

} /* close namespace */
#endif /* UNIFORMGRID_HH_ */
