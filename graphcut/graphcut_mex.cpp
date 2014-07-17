#include "mex.h"
#include <math.h>

#include "graphcut_mex.h"
#include "graph.h"
#include <iostream> 
#include <vector>
#include <utility>
#include <queue>
#include <set>
#include <algorithm>

typedef Graph<double,double,double> GraphType;

using namespace std;



///////////////////////
// GLOBAL PARAMETERS //
///////////////////////
const int * DIMS;
int NDIM;
int NUM_PIXELS; // number of pixels in image

double WEIGHT_EXPONENT; 


int toLinear2D(int row, int col){
    return row + DIMS[0]*col;
}
// get neighbouring pixels (including diagonals)
vector<int> getNeighbours2D(int n){
    int H = DIMS[0];
    int W = DIMS[1];
    int row = n % H;
    int col = n / H;
    vector<int> v;
    
    for (int i = max(row-1, 0); i <= min(row+1, H-1); i++){
        for (int j = max(col-1,0); j <= min(col+1, H-1); j++){
            if (! (i == row && j == col)){
                v.push_back(toLinear2D(i, j));
            }
        }
    }
    return v;
}

int toLinear3D(int row, int col, int depth){
    return (row + DIMS[0]*col) + (DIMS[0]*DIMS[1]) * depth;
}
// get neighbouring pixels (in 26-connected grid)
vector<int> getNeighbours3D(int n){
    int H = DIMS[0];
    int W = DIMS[1];
    int D = DIMS[2];
    int linIndex = n % (H*W);
    int depth = n / (H*W);
    int row = linIndex % H;
    int col = linIndex / H;
    vector<int> v;
    
    for (int i = max(row-1, 0); i <= min(row+1, H-1); i++){
        for (int j = max(col-1,0); j <= min(col+1, W-1); j++){
            for (int k = max(depth-1,0); k <= min(depth+1, D-1); k++){
                if (! (i == row && j == col && k == depth)){
                    v.push_back(toLinear3D(i, j, k));
                }
            }
        }
    }
    return v;
}

vector<int> getNeighbours(int n){
    if (NDIM == 2) return getNeighbours2D(n);
    else if (NDIM == 3) return getNeighbours3D(n);
}

int numEdges(){
    if (NDIM == 2) NUM_PIXELS * 8;
    else NUM_PIXELS * 26;
}


double edgeWeight(double u, double v){
    double diff = fabs(u-v);
    if (diff < 0.001) return INFINITY;
    return 1 / pow(diff,WEIGHT_EXPONENT);    
}

// builds an adjacency matrix for the given array
GraphType* buildGraph(const double * pixels, const double * sourceWeights, const double * sinkWeights){
    GraphType *g = new GraphType(/*estimated # of nodes*/ NUM_PIXELS, /*estimated # of edges*/ numEdges()); 
    
    for (int i = 0; i < NUM_PIXELS; i++){
        g -> add_node();
        g -> add_tweights(i, sourceWeights[i], sinkWeights[i]);
    }
    
    for (int i = 0; i < NUM_PIXELS; i++){
        vector<int> neighbours = getNeighbours(i);
        for (int j = 0; j < neighbours.size(); j++){
            int n = neighbours[j];
            if (n <= j) continue; //don't double-insert edges
            
            g -> add_edge(i, n, edgeWeight(pixels[i], pixels[n]), edgeWeight(pixels[n], pixels[i]));
        }        
    } 
     
    return g;
}


    


///////////////////////
// MAIN ENTRY POINT  //
///////////////////////

void mexFunction(
    int		  nlhs, 	/* number of expected outputs */
    mxArray	  *plhs[],	/* mxArray output pointer array */
    int		  nrhs, 	/* number of inputs */
    const mxArray	  *prhs[]	/* mxArray input pointer array */
    )
{
    if ( nrhs != 4 )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","Must have four inputs");
    if ( mxGetClassID(prhs[0]) != mxDOUBLE_CLASS )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","image must be of type double");
    if ( mxGetClassID(prhs[1]) != mxDOUBLE_CLASS )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","src must be of type double");
    if ( mxGetClassID(prhs[2]) != mxDOUBLE_CLASS )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","sink must be of type double");
    
    /* first argument - the (2D) image */
    double * pixels = (double*)mxGetData(prhs[0]);
    DIMS = mxGetDimensions(prhs[0]);
    NDIM = mxGetNumberOfDimensions(prhs[0]);
   
    NUM_PIXELS = 1;
    for (int i = 0; i < NDIM; i++){
        NUM_PIXELS *= DIMS[i];
    }
    
    /* next two arguments - the source and sink weights */
    double * sourceWeights = (double*)mxGetData(prhs[1]);
    double * sinkWeights = (double*)mxGetData(prhs[2]);
    
    /* next argument - weight function parameters */
    WEIGHT_EXPONENT = (double) mxGetScalar(prhs[3]);
    
    /* second input - parameters structure *  

    GetScalar(mxGetField(prhs[1], 0, "hi"), HI);
    */
    
    
    
    
    // first output - the feature space raw image
    plhs[0] = mxCreateNumericArray(NDIM, DIMS, mxLOGICAL_CLASS, mxREAL);
    bool* outImage = (bool*)mxGetData(plhs[0]);
    for (int i = 0; i < NUM_PIXELS; i++){
        outImage[i] = false;
    }
    
    
    

    GraphType * g = buildGraph(pixels, sourceWeights, sinkWeights);
    g -> maxflow();

    
    
     // create object array
    for (int i = 0; i< NUM_PIXELS; i++){
        outImage[i] = g->what_segment(i) == GraphType::SOURCE;
    }
    
    delete g;
   
}

template<class T>
void GetScalar(const mxArray* x, T& scalar)
{
    if ( mxGetNumberOfElements(x) != 1 )
        mexErrMsgIdAndTxt("weight_sample_mex:GetScalar","input is not a scalar");
    void *p = mxGetData(x);
    switch (mxGetClassID(x)) {
        case mxLOGICAL_CLASS:
            scalar = *(bool*)p;
            break;
        case mxCHAR_CLASS:
            scalar = *(char*)p;
            break;
        case mxDOUBLE_CLASS:
            scalar = *(double*)p;
            break;
        case mxSINGLE_CLASS:
            scalar = *(float*)p;
            break;
        case mxINT8_CLASS:
            scalar = *(char*)p;
            break;
        case mxUINT8_CLASS:
            scalar = *(unsigned char*)p;
            break;
        case mxINT16_CLASS:
            scalar = *(short*)p;
            break;
        case mxUINT16_CLASS:
            scalar = *(unsigned short*)p;
            break;
        case mxINT32_CLASS:
            scalar = *(int*)p;
            break;
        case mxUINT32_CLASS:
            scalar = *(unsigned int*)p;
            break;
#ifdef A64BITS            
        case mxINT64_CLASS:
            scalar = *(int64_T*)p;
            break;
        case mxUINT64_CLASS:
            scalar = *(uint64_T*)p;
            break;
#endif /* 64 bits machines */            
        default:
            mexErrMsgIdAndTxt("GraphCut:GetScalar","unsupported data type");
    }
}

