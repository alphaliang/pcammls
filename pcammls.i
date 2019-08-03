%module pcammls


%{
#include "TYApi.h"
#include "TYImageProc.h"
#include "TyIsp.h"
%}


%include "stdint.i"

#define TY_STATIC_LIB 1
#define TY_EXTC 

%include "TYApi.h"
%include "TYImageProc.h"
%include "TyIsp.h"


