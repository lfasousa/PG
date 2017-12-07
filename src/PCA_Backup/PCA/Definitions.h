#ifndef DEFINITIONS_H
#define DEFINITIONS_H

/*
Parameter for the refinement of the initial landmark-based alignment using a rigid ICP
*/

//Number of ICP alignment iterations.
const int ICP_NUMBER_ALIGNMENT_ITERATIONS = 100;
//Maximum point distance for ICP (unit of the learned model).
const double ICP_MAX_POINT_DISTANCE = 0.10;


/*
Parameter for the fitting
*/

//Size of the prior-box of the second mode (distance from the mean within the second mode that are allowed while fitting).
const double PROJECTION_MODE2_PRIOR_BOX_SIZE = 1.0;
//Size of the prior-box of the third mode (distance from the mean within the third mode that are allowed while fitting).
const double PROJECTION_MODE3_PRIOR_BOX_SIZE = 0.0;

//Maximum point distance for fitting (unit of the learned model).
const double PROJECTION_MAX_POINT_DISTANCE = 0.10;

//If enabled, non-linear optimization outputs function and gradient in each step.
#define TRACE_NONLINEAR_PROJECTION_METHOD

//se ativado, modificações manuais serão ativadas no minimizer vnl_lbfgsb
#define MODIFICACOES_MANUAIS

#endif