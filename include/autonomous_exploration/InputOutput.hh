/*
 * InputOutput.hh
 *
 *     created: Jan 2017
 *      author: Matthias Rungger
 */

/** @file **/

#ifndef InputOutput_HH_
#define InputOutput_HH_

#include <string>

#include "FileHandler.hh"
#include "UniformGrid.hh"
#include "TransitionFunction.hh"
#include "StaticController.hh"
#include "WinningDomain.hh"

#ifdef SCOTS_BDD
#include "SymbolicSet.hh"
#endif 

/* StaticController definitions */
#define  SCOTS_SC_TYPE          "STATICCONTROLLER"

/* WinningDomain definitions */
#define  SCOTS_WD_TYPE          "WINNINGDOMAIN"
#define  SCOTS_WD_DATA          "DATA"

/* UniformGrid definitions */
#define SCOTS_UG_TYPE         "UNIFORMGRID"
#define SCOTS_UG_DIM          "DIM"
#define SCOTS_UG_ETA          "ETA"
#define SCOTS_UG_LOWER_LEFT   "LOWER_LEFT"
#define SCOTS_UG_UPPER_RIGHT  "UPPER_RIGHT"

/* SymbolicSet definitions */
#define SCOTS_SS_TYPE         "SYMBOLICSET"
#define SCOTS_SS_BDD_VAR_ID   "BDD_VAR_ID_IN_DIM_"

/* TransitionFunction definitions */
#define SCOTS_TF_TYPE         "TRANSITIONFUNCTION"
#define SCOTS_TF_NO_STATES    "NO_STATES"
#define SCOTS_TF_NO_INPUTS    "NO_INPUTS"
#define SCOTS_TF_NO_TRANS     "NO_TRANS"
#define SCOTS_TF_NO_PRE       "NO_PRE"
#define SCOTS_TF_NO_POST      "NO_POST"
#define SCOTS_TF_PRE_PTR      "PRE_PTR"
#define SCOTS_TF_PRE          "PRE"

/* Grid points definitions */
#define  SCOTS_GP_TYPE        "SET_OF_GRIDPOINTS"
#define  SCOTS_GP_DATA        "GRIDPOINTS"

/** @namespace scots **/ 
namespace scots {

/** @brief write WinningDomain to a file via a FileWriter **/
inline
bool write_to_file(const WinningDomain& wd, const std::string& filename, bool append_to_file=false ) {
    FileWriter writer(filename);
    if(append_to_file) {
        if(!writer.open()) {
            return false;
        }
    } else {
        if(!writer.create()) {
            return false;
        }
    }
    writer.add_TYPE(SCOTS_WD_TYPE);
    writer.add_TEXT("i (state) j_0 ... j_n (valid inputs)");
    writer.add_WINNINGDOMAIN(SCOTS_WD_DATA,
                             wd.m_winning_domain,
                             wd.m_inputs,
                             wd.m_no_states,
                             wd.m_no_inputs);
    writer.close();
    return true;
}

/** @brief write StaticController to a file via a FileWriter **/
inline
bool write_to_file(const StaticController& sc, const std::string& filename, bool append_to_file = false) {
    FileWriter writer(filename);
    if(append_to_file) {
        if(!writer.open()) {
            return false;
        }
    } else {
        if(!writer.create()) {
            return false;
        }
    }
        writer.add_VERSION();
        writer.add_TYPE(SCOTS_SC_TYPE);

        /* write UniformGrid information of the state space */
        writer.add_TEXT("STATE_SPACE");
        writer.add_TYPE(SCOTS_UG_TYPE);
        writer.add_MEMBER(SCOTS_UG_DIM,sc.m_state_grid.get_dim());
        writer.add_VECTOR(SCOTS_UG_ETA,sc.m_state_grid.get_eta());
        writer.add_VECTOR(SCOTS_UG_LOWER_LEFT,sc.m_state_grid.get_lower_left());
        writer.add_VECTOR(SCOTS_UG_UPPER_RIGHT,sc.m_state_grid.get_upper_right());

        /* write UniformGrid information of the input space */
        writer.add_TEXT("INPUT_SPACE");
        writer.add_TYPE(SCOTS_UG_TYPE);
        writer.add_MEMBER(SCOTS_UG_DIM,sc.m_input_grid.get_dim());
        writer.add_VECTOR(SCOTS_UG_ETA,sc.m_input_grid.get_eta());
        writer.add_VECTOR(SCOTS_UG_LOWER_LEFT,sc.m_input_grid.get_lower_left());
        writer.add_VECTOR(SCOTS_UG_UPPER_RIGHT,sc.m_input_grid.get_upper_right());

        writer.close();

        /* write WinningDomain */
        return write_to_file(sc.m_winning_domain,filename,true);
}

/** @brief write TransitionFunction to a file via a FileWriter **/
inline
bool write_to_file(const TransitionFunction& tf, const std::string& filename) {
    FileWriter writer(filename);
    if(writer.create()) {
        abs_type N=tf.m_no_states;
        abs_type M=tf.m_no_inputs;
        abs_type T=tf.m_no_transitions;

        writer.add_VERSION();
        writer.add_TYPE(SCOTS_TF_TYPE);
        writer.add_MEMBER(SCOTS_TF_NO_STATES,N);
        writer.add_MEMBER(SCOTS_TF_NO_INPUTS,M);
        writer.add_MEMBER(SCOTS_TF_NO_TRANS,T);
        writer.add_ARRAY(SCOTS_TF_NO_PRE,tf.m_no_pre,N*M);
        writer.add_ARRAY(SCOTS_TF_NO_POST,tf.m_no_post,N*M);
        writer.add_ARRAY(SCOTS_TF_PRE_PTR,tf.m_pre_ptr,N*M);
        writer.add_ARRAY(SCOTS_TF_PRE,tf.m_pre,T);

        writer.close();
        return true;
    }
    return false;
}

/** @brief write UniformGrid to a file via a FileWriter **/
inline
bool write_to_file(const UniformGrid& grid, const std::string& filename, bool append_to_file = false) {
    FileWriter writer(filename);
    if(append_to_file){
        if(!writer.open()){
            return false;
        }
    }
    else if(!writer.create()){
        return false;
    }

    writer.add_VERSION();
    writer.add_TYPE(SCOTS_UG_TYPE);
    writer.add_MEMBER(SCOTS_UG_DIM,grid.get_dim());
    writer.add_VECTOR(SCOTS_UG_ETA,grid.get_eta());
    writer.add_VECTOR(SCOTS_UG_LOWER_LEFT,grid.get_lower_left());
    writer.add_VECTOR(SCOTS_UG_UPPER_RIGHT,grid.get_upper_right());

    writer.close();
    return true;
}


/** @brief write atomic propositions to file **/
template<class F>
bool write_to_file(const UniformGrid& grid, const F& atomic_prop, const std::string& filename) {
    FileWriter writer(filename);

    /* store grid information */
    if(!write_to_file(grid,filename)) {
        return false;
    }
    std::vector<abs_type> gp {};
    for(abs_type i=0; i<grid.size(); i++) {
        if(atomic_prop(i)) {
            gp.push_back(i);
        }
    }
    if(!writer.open()) {
        return false;
    }
    if(!writer.add_TYPE(SCOTS_GP_TYPE)) {
        return false;
    }
    if(!writer.add_VECTOR(SCOTS_GP_DATA,gp)) {
        return false;
    }
    writer.close();
    return true;
}

#ifdef SCOTS_BDD
/** @brief write SymbolicSet to file **/
inline
bool write_to_file(const SymbolicSet& set, const std::string& filename) {
    FileWriter writer(filename);
    if(writer.create()) {

        writer.add_VERSION();
        writer.add_TYPE(SCOTS_SS_TYPE);
        writer.add_MEMBER(SCOTS_UG_DIM,set.get_dim());
        writer.add_VECTOR(SCOTS_UG_ETA,set.get_eta());
        writer.add_VECTOR(SCOTS_UG_LOWER_LEFT,set.get_lower_left());
        writer.add_VECTOR(SCOTS_UG_UPPER_RIGHT,set.get_upper_right());
        auto intervals = set.get_bdd_intervals();
        for(int i=0;i<set.get_dim(); i++) {
            std::string s = SCOTS_SS_BDD_VAR_ID;
            writer.add_VECTOR(s.append(std::to_string(i+1)),intervals[i].get_bdd_var_ids());
        }

        writer.close();
        return true;
    }
    return false;
}

/** 
 * @brief write BDD with its SymbolicSet information to a file via a FileWriter
 * The BDD is stored in filename.bdd\n
 * The SymbolicSet is stored in filename.scs\n
 * See dddmp.h for the different modes of writing.
 **/
inline
bool write_to_file(const Cudd& manager, const SymbolicSet& set, const BDD& bdd, const std::string& filename, char mode='B') {
    FileWriter writer(filename);
    if(!write_to_file(set,filename)) {
        return false;
    }
   /* check if we should write slugs variable names */
   if(set.get_slugs_var_names().size()) {
    /* create data structure for variable names of the BDD variables */
    char** varnames = new char*[manager.ReadSize()];
    for(int i=0; i<manager.ReadSize(); i++) 
      varnames[i]=nullptr;
    /* create marker to keep track which variable names are allocated here */
    std::unique_ptr<bool[]> marker(new bool[manager.ReadSize()]());
     /* fill varnames with slugs variable names */
    auto slugs_names = set.get_slugs_var_names();
    auto bdd_ids = set.get_bdd_var_ids();
    for(size_t i=0; i<slugs_names.size(); i++) 
      varnames[bdd_ids[i]]=&slugs_names[i][0];
    for(int i=0; i<manager.ReadSize(); i++) {
      if(varnames[i]==nullptr) {
        varnames[i] = new char;
        varnames[i][0]='d';
        marker[i]=true;
      }
    }

    /* write BDD to file */
    if(!writer.add_BDD(manager,bdd,varnames,mode)) {
        return false;
    }

    /* clean variable names */
    for(int i=0; i<manager.ReadSize(); i++) {
      if(marker[i])
        delete varnames[i];
    }
    delete[] varnames;
   } else {
    /* write BDD to file */
    if(!writer.add_BDD(manager,bdd,nullptr,mode)) {
        return false;
    }
  }

  return true;
}
#endif


/** @brief read WinningDomain from a file via a FileReader **/
inline
bool read_from_file(WinningDomain& wd, const std::string& filename, size_t offset=0) {
    FileReader reader(filename);
    if(!reader.open()) {
        return false;
    }
    abs_type N;
    abs_type M;
    std::vector<bool> inputs{};
    std::vector<abs_type> domain{};
    if(!reader.get_WINNINGDOMAIN(SCOTS_WD_DATA,domain,inputs,N,M,offset)) {
        return false;
    }
    wd = WinningDomain(N,M,std::move(domain),std::move(inputs),std::numeric_limits<abs_type>::max());
    return true;
}

/** @brief read UniformGrid from a file via a FileReader **/
inline
bool read_from_file(UniformGrid& grid, const std::string& filename, size_t offset = 0) {
    FileReader reader(filename);
    if(!reader.open()) {
        return false;
    }
    int dim;
    if(!reader.get_MEMBER(SCOTS_UG_DIM,dim,offset)) {
        return false;
    }
    std::vector<double> eta;
    if(!reader.get_VECTOR(SCOTS_UG_ETA,eta,offset)) {
        return false;
    }
    std::vector<double> lb;
    if(!reader.get_VECTOR(SCOTS_UG_LOWER_LEFT,lb,offset)) {
        return false;
    }
    std::vector<double> ub;
    if(!reader.get_VECTOR(SCOTS_UG_UPPER_RIGHT,ub,offset)) {
        return false;
    }
    reader.close();
    /* make sure that rounding in the UniformGrid constructor works correctly */
    for(int i=0; i<dim; i++) {
        lb[i]-=eta[i]/4.0;
        ub[i]+=eta[i]/4.0;
    }
    grid = UniformGrid(dim,lb,ub,eta);
    return true;
}

/** @brief read StaticController from a file via a FileReader **/
inline
bool read_from_file(StaticController& sc, const std::string& filename) {
    FileReader reader(filename);
    if(!reader.open()) {
        return false;
    }
    /* find offset where the UniformGrid of the state space is stored */
    size_t ss_offset=reader.find_TEXTPOS("STATE_SPACE");
    size_t is_offset=reader.find_TEXTPOS("INPUT_SPACE");
    if(!ss_offset || !is_offset) {
        return false;
    }
    reader.close();
    /* read UniformGrid info */
    UniformGrid ss;
    if(!read_from_file(ss,filename,ss_offset)) {
        return false;
    }
    /* read UniformGrid info */
    UniformGrid is;
    if(!read_from_file(is,filename,is_offset)) {
        return false;
    }
    /* read WinningDomain  info */
    WinningDomain wd;
    if(!read_from_file(wd,filename)) {
        return false;
    }
    sc = StaticController(ss,is,std::move(wd));
    return true;
}

/** @brief read TransitionFunction from a file via a FileReader **/
inline
bool read_from_file(TransitionFunction& tf, const std::string& filename) {
    FileReader reader(filename);
    if(!reader.open()) {
        return false;
    }
    std::string type;
    if(!reader.get_TYPE(type)) {
        return false;
    }
    if(type!=SCOTS_TF_TYPE) {
        return false;
    }
    abs_type N;
    if(!reader.get_MEMBER(SCOTS_TF_NO_STATES,N)) {
        return false;
    }
    abs_type M;
    if(!reader.get_MEMBER(SCOTS_TF_NO_INPUTS,M)) {
        return false;
    }
    abs_ptr_type T;
    if(!reader.get_MEMBER(SCOTS_TF_NO_TRANS,T)) {
        return false;
    }
    tf.init_infrastructure(N,M);
    tf.init_transitions(T);
    size_t offset;
    offset=reader.get_ARRAY(SCOTS_TF_NO_PRE,tf.m_no_pre,N*M);
    std::cout << "off " <<offset << "\n";
    if(!offset) {
        tf.clear();
        return false;
    }
    offset=reader.get_ARRAY(SCOTS_TF_NO_POST,tf.m_no_post,N*M,offset);
    std::cout << "off " <<offset << "\n";
    if(!offset) {
        tf.clear();
        return false;
    }
    offset=reader.get_ARRAY(SCOTS_TF_PRE_PTR,tf.m_pre_ptr,N*M,offset);
    std::cout << "off " <<offset << "\n";
    if(!offset) {
        tf.clear();
        return false;
    }
    offset=reader.get_ARRAY(SCOTS_TF_PRE,tf.m_pre,T,offset);
    std::cout << "off " <<offset << "\n";
    if(!offset) {
        tf.clear();
        return false;
    }
    reader.close();
    return true;
}

#ifdef SCOTS_BDD
/** @brief read SymbolicSet to file **/
inline
bool read_from_file(const Cudd& manager, SymbolicSet& set, const std::string& filename) {
    /* read UniformGrid from file */
    UniformGrid grid;
    if(!read_from_file(grid,filename)){
        return false;
    }
    /* read the BDD variable IDs and create IntegerInterval*/
    std::vector<IntegerInterval<abs_type>> bdd_interval{};
    FileReader reader(filename);
    if(reader.open()) {
        size_t offset = 0;
        for(int i=0;i<grid.get_dim(); i++) {
            std::vector<unsigned int> var_id {};
            std::string s = SCOTS_SS_BDD_VAR_ID;
            offset=reader.get_VECTOR(s.append(std::to_string(i+1)),var_id,offset);
            if(!offset)
                return false;
            bdd_interval.emplace_back(manager,abs_type{0},grid.get_no_gp_per_dim()[i]-abs_type{1},var_id);
        }
        reader.close();
    } else {
        return false;
    }
    /* initialize SymbolicSet  */
    set = SymbolicSet(grid,bdd_interval);
    return true;
}

/** 
 * @brief read BDD with its SymbolicSet information from file
 * The SymbolicSet is read from filename.scs\n
 * The BDD is read from filename.bdd\n
 * See dddmp.h for the different modes of reading.
 **/
inline
bool read_from_file(const Cudd& manager, SymbolicSet& set, BDD& bdd, const std::string& filename, char mode = 'B') {
    if(!read_from_file(manager,set,filename))
        return false;
    FileReader reader(filename);
    if(!reader.get_BDD(manager,bdd,mode)) {
        return false;
    }
    set.clean(manager,bdd);
    return true;
}
#endif


} /* end namespace */
#endif /* InputOutput_HH_ */
