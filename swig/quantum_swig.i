/* -*- c++ -*- */

#define QUANTUM_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "quantum_swig_doc.i"


// So we understand the firdes window types
%import "gnuradio/filter/firdes.h"


%{
#include "quantum/controllers_Coprocessor.h"
#include "quantum/controllers_Analyzer.h"
#include "quantum/controllers_Generator.h"
#include "quantum/controllers_readout.h"
#include "quantum/controllers_Initializer.h"
#include "quantum/controllers_gatesParams.h"
#include "quantum/controllers_sync.h"
#include "quantum/controllers_wait.h"
#include "quantum/controllers_callback_message.h"
#include "quantum/gates_CNOT.h"
#include "quantum/gates_H.h"
#include "quantum/gates_junction.h"
#include "quantum/gates_S.h"
#include "quantum/gates_T.h"
#include "quantum/gates_X.h"
#include "quantum/gates_Y.h"
#include "quantum/gates_Z.h"
#include "quantum/measurements_Detector.h"
#include "quantum/API_OpenQL_Source.h"
#include "quantum/API_OpenQL_Sink.h"
%}

%include "quantum/controllers_Coprocessor.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_Coprocessor);
%include "quantum/controllers_Initializer.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_Initializer);
%include "quantum/controllers_readout.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_readout);
%include "quantum/controllers_Analyzer.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_Analyzer);
%include "quantum/controllers_Generator.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_Generator);
%include "quantum/controllers_gatesParams.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_gatesParams);
%include "quantum/controllers_sync.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_sync);
%include "quantum/controllers_wait.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_wait);
%include "quantum/controllers_callback_message.h"
GR_SWIG_BLOCK_MAGIC2(quantum, controllers_callback_message);
%include "quantum/gates_CNOT.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_CNOT);
%include "quantum/gates_H.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_H);
%include "quantum/gates_junction.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_junction);
%include "quantum/gates_S.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_S);
%include "quantum/gates_T.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_T);
%include "quantum/gates_X.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_X);
%include "quantum/gates_Y.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_Y);
%include "quantum/gates_Z.h"
GR_SWIG_BLOCK_MAGIC2(quantum, gates_Z);
%include "quantum/measurements_Detector.h"
GR_SWIG_BLOCK_MAGIC2(quantum, measurements_Detector);
%include "quantum/API_OpenQL_Source.h"
GR_SWIG_BLOCK_MAGIC2(quantum, API_OpenQL_Source);
%include "quantum/API_OpenQL_Sink.h"
GR_SWIG_BLOCK_MAGIC2(quantum, API_OpenQL_Sink);
