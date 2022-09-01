/*
 * scots.hpp
 *
 *     created: Jan 2017
 *      author: Matthias Rungger
 */

/** @file **/

#ifndef SCOTS_HPP_
#define SCOTS_HPP_

#include "autonomous_exploration/TransitionFunction.hpp"
#include "autonomous_exploration/UniformGrid.hpp"
#include "autonomous_exploration/Abstraction.hpp"
#include "autonomous_exploration/GameSolver.hpp"
#include "autonomous_exploration/WinningDomain.hpp"
#include "autonomous_exploration/StaticController.hpp"
#include "autonomous_exploration/InputOutput.hpp"


/* if scots is used in connection with the cudd library */
#ifdef  SCOTS_BDD
/* cudd library */
#include "dddmp.h"
/* scots classes with bdd support */
#include "SymbolicSet.hh"
#include "SymbolicModel.hh"
#include "EnfPre.hh"
#endif

#endif /* SCOTS_HPP_ */

