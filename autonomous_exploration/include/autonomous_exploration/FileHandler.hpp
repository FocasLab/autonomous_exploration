/*
 * FileHandler.hh
 *
 *     created: Dec 2016
 *      author: Frederik Kunik
 */


#ifndef FILEHANDLER_HPP_
#define FILEHANDLER_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <climits>
#include <iomanip>

#define SCOTS_FH_VERSION    "v0.2"
#define SCOTS_FH_SYMBOL     "#"
#define SCOTS_FH_SEPERATOR  ";"
#define SCOTS_FH_EXTENSION  ".scs"
#define SCOTS_FH_BDD_EXTENSION  ".bdd"

#define SCOTS_FH_KEY        "#SCOTS:"
#define SCOTS_FH_TYPE       "#TYPE:"
#define SCOTS_FH_MEMBER     "#MEMBER:"
#define SCOTS_FH_VECTOR     "#VECTOR:"
#define SCOTS_FH_ARRAY      "#ARRAY:"
#define SCOTS_FH_MATRIX     "#MATRIX:"
#define SCOTS_FH_BEGIN      "#BEGIN:"
#define SCOTS_FH_END        "#END"

namespace scots {
  
  
/** @cond **/

/* The FileHandler class stores the filename */
class FileHandler {
protected:
  std::string m_filename{};
public:
  FileHandler(const std::string& filename) : m_filename(filename) {};

  std::string get_filename() {
    return m_filename;
  }
};

/* The FileWriter class is used to write information into files */
class FileWriter : public FileHandler {
private:
  std::ofstream m_file;
public:
  FileWriter(const std::string& filename) : FileHandler(filename) {};
  bool create() {
    m_file.close();
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION),std::fstream::out);
    return m_file.is_open();
  }
  bool open() {
    m_file.close();
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION),std::fstream::app);
    return m_file.is_open();
  }
  void close() {
    m_file.close();
  }
  bool add_TEXT(const std::string& text) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_KEY << text <<"\n";
      return true;
    }
    return false;
  }
  bool add_VERSION(void) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_KEY << SCOTS_FH_VERSION <<"\n";
      return true;
    }
    return false;
  }
  bool add_TYPE(const std::string& type) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_TYPE << type << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_MEMBER(const std::string& name,T &&member) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_MEMBER << name << "\n";
      m_file << member << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_VECTOR(const std::string& name, const std::vector<T>& vector) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_VECTOR << name << "\n";
      m_file << SCOTS_FH_BEGIN << vector.size() << "\n";
      m_file <<  std::setprecision(std::numeric_limits<T>::max_digits10);
      for(size_t index = 0; index < vector.size(); index++) {
        m_file << vector[index] << "\n";
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_ARRAY(const std::string& name, T&& array, size_t array_size) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_ARRAY << name << "\n";
      m_file << SCOTS_FH_BEGIN << array_size << "\n";
      for(size_t index = 0; index < array_size; index++) {
        m_file << array[index] << "\n";
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_WINNINGDOMAIN(const std::string& name, 
                         const std::vector<T>& vector,
                         const std::vector<bool>& matrix,
                         size_t row, size_t col) {
    if(vector.size()!=row) {
      return false;
    }
    if(m_file.is_open()) {
      m_file << SCOTS_FH_MATRIX << name << "\n";
      if(matrix.size()==row*col) {
        m_file << SCOTS_FH_BEGIN << row << " " << col << "\n";
        for(size_t i=0; i<row; i++) {
          if(vector[i]!=std::numeric_limits<T>::max()) {
            m_file << i << " ";
            for(size_t j=0; j<col; j++) {
              if(matrix[i*col+j]) {
                m_file << j << " ";
              }
            }
            m_file << "\n";
          }
        }
      } else {
        m_file << SCOTS_FH_BEGIN << row << " " << 1 << "\n";
        for(size_t i=0; i<row; i++) {
          if(vector[i]!=std::numeric_limits<T>::max()) {
            m_file << i << " " << vector[i] << "\n";
          }
        }
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
};

/* The FileReader class is used to read information from files */
class FileReader : public FileHandler {
private:
  std::ifstream       m_file;
  std::string         m_line;

  bool skip_offset(size_t& offset) {
    for(size_t index = 0; index<offset; index++) {
      if(!(m_file.ignore(std::numeric_limits<std::streamsize>::max(),'\n'))) {
        return false;
      }
    }
    return true;
  }
  void back_to_first_line() {
    m_file.clear();
    m_file.seekg(0,std::ios::beg);
  }

public:
  FileReader(const std::string& filename) : FileHandler(filename) {};
  bool open() {
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION));
    return m_file.good();
  }
  void close() {
    m_file.close();
  }
  size_t get_VERSION(double& version, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(SCOTS_FH_KEY)!=std::string::npos) {
          std::istringstream stream(m_line.substr(m_line.find(":")+1));
          stream >> version;
          return counter;
        }
      }
      return 0;
    }
    return 0;
  }
  size_t get_TYPE(std::string& string, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      string.clear();
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(SCOTS_FH_TYPE)!=std::string::npos) {
          std::istringstream stream(m_line.substr(m_line.find(":")+1));
          stream >> string;
          return counter;
        }
      }
    }
    return 0;
  }
  size_t find_TEXTPOS(const std::string& text, size_t offset=0) {
    std::string match = SCOTS_FH_KEY + text;
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line==match) {
          return counter;
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_MEMBER(const std::string& member_name, T& member, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      std::string match = SCOTS_FH_MEMBER + member_name;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            std::istringstream stream(m_line);
            stream >> member;
            counter++;
            return counter;
          }
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_VECTOR(const std::string& vector_name, std::vector<T>& vector, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t size=0;
      size_t counter=0;
      std::string match = SCOTS_FH_VECTOR + vector_name;
      vector.clear();
      /* first get the size of the vector */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> size;
              counter++;
              break;
            }
          }
        }
      }
      if(m_file.eof() || size==0) {
        return 0;
      }
      /* now fill the vector */
      vector.resize(size);
      for(size_t index = 0; index < size; index++) {
        if(std::getline(m_file,m_line)) {
          if(m_line.find(SCOTS_FH_SYMBOL)!=std::string::npos) {
            vector.clear();
            return 0;
          }
          counter++;
          std::istringstream stream(m_line);
          stream >> vector[index];
        } else {
          vector.clear();
          return 0;
        }
      }
      if(std::getline(m_file,m_line)) {
        if(m_line.find(SCOTS_FH_END)!=std::string::npos) {
          counter++;
          return counter;
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_ARRAY(const std::string array_name, T& array, size_t array_size, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t size=0;
      size_t counter=0;
      std::string match = SCOTS_FH_ARRAY + array_name;
      /* check size */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> size;
              counter++;
              break;
            } 
          }
        }
      }
      if(m_file.eof() || size==0 || size!=array_size) {
        return 0;
      }
      /* fill array */
      for(size_t index = 0; index < size; index++) {
        if(std::getline(m_file,m_line)) {
          counter++;
          std::istringstream stream(m_line);
          stream >> array[index];
        } else {
          return 0;
        }
      }
      if(std::getline(m_file,m_line)) {
        if(m_line.find(SCOTS_FH_END)!=std::string::npos) {
          counter++;
          return counter+offset;
        }
      }
    }
    return 0;
  }
  template<class T>
  bool get_WINNINGDOMAIN(const std::string& name, 
                         std::vector<T>& vector,
                         std::vector<bool>& matrix,
                         T& row, 
                         T& col,
                         size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      std::string match = SCOTS_FH_MATRIX + name;
      /* get size */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line==match) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> row >> col;
              counter++;
              break;
            }
          }
        }
      }
      /* nothing found */
      if(m_file.eof()) {
        return 0;
      }
      /* if col > 1 the boolean matrix[row][col] is necessary to represent the winning domain */
      if(col>1) {
        matrix.clear();
        matrix.resize(row*col,false);
      }
      vector.clear();
      vector.resize(row,std::numeric_limits<T>::max());
      T i,j;
      /* fill vector and matrix */
      while(std::getline(m_file,m_line)) {
        counter++;
        /* check if we reached end */
        if(m_line==SCOTS_FH_END) {
          return counter+offset;
        }
        std::istringstream stream(m_line);
        stream >> i;
        if(col>1) {
          while( stream >> j ) {
            matrix[i*col+j]=true;
            vector[i]=0;
          }
        } else {
          stream >> j;
          vector[i]=j;
        }
      }
    }
    return 0;
  }
};


/** @endcond **/

} /*end of namepace scots*/
#endif /* FILEHANDLER_HPP_ */
