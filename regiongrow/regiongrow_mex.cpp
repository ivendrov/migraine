#include "mex.h"
#include <math.h>

#include "regiongrow_mex.h"
#include <iostream> 
#include <vector>
#include <utility>
#include <queue>
#include <set>
#include <algorithm>

using namespace std;


#define edge pair<int, double>
#define ve vector<edge >
#define adj vector<ve >



///////////////////////
// GLOBAL PARAMETERS //
///////////////////////
int H; // height of image
int W; // width of image
int NUM_PIXELS; // number of pixels in image

/* Explanation: a pixel is in the foreground iff
 * there is a path from a seed to it that never involves 
 *  (a) a step that goes "hi" uphill 
 *  (b) a series of consecutive steps greater than "lo"
 *      adding up to more than "sumlo"
 */
double HI; 
double LO;
double SUMLO;

void printEdges(ve & edges){
    for (int i = 0; i < edges.size(); i++){
        edge e = edges[i];
        printf("Edge to %i of length %f\n", e.first, e.second);
    }
}


int toLinear(int row, int col){
    return row + H*col;
}
// get neighbouring pixels (in 4-connected grid)
vector<int> getNeighbours(int n){
    int row = n % H;
    int col = n / H;
    vector<int> v;
    
    for (int i = max(row-1, 0); i <= min(row+1, H-1); i++){
        for (int j = max(col-1,0); j <= min(col+1, H-1); j++){
            if (! (i == row && j == col)){
                v.push_back(toLinear(i, j));
            }
        }
    }
    return v;
}


// builds an adjacency matrix for the given array
adj buildAdj(const double * pixels){
    adj A; 
    A.resize(NUM_PIXELS);
    for (int i = 0; i < NUM_PIXELS; i++){
        vector<int> neighbours = getNeighbours(i);
        ve edges;
        for (int j = 0; j < neighbours.size(); j++){
            int n = neighbours[j];
            double diff = pixels[n] - pixels[i];
            if (diff < 0) diff = diff * (-0.4);
            if (diff <= HI){ // never go more than HI in ANY direction
                if (diff <= LO) diff = 0; // less than LO uphill doesn't count
                edges.push_back(make_pair(n,diff));
            }                
            //printf("Edge from %i to %i with gradient %F\n", i, n, diff);
        }
        A[i] = edges;
    }
    return A;
}


    

// uses Dijkstra's algorithm to find all pixels within SUMLO
// of any given seed pixel
void dijkstra(adj & A, vector<int> & seeds, bool * object){
 
    
    // initialize distances
    double * distance = new double[NUM_PIXELS];
    for (int i = 0; i < NUM_PIXELS; i++){     
       distance[i] = INFINITY;
    }
    for (int i = 0; i < seeds.size(); i++){
        distance[seeds[i]] = 0;
    }
    
    // initialize queue
    set<pair<double, int> > Q;
    for (int i = 0; i< seeds.size(); i++){
        Q.insert(make_pair(0, seeds[i]));
    }
    
    
    
    // run Dijkstra's
    // INVARIANT: Q only contains object pixels that have not 
    // yet been expanded
    while (!Q.empty()){
        int cur = Q.begin() -> second;
        double curD = Q.begin() -> first;
        Q.erase(Q.begin());
        //printf("Expanding %i (%i, %i)\n", cur, cur%H, cur/H);
        ve neighbours = A[cur];
        for (int i = 0; i < neighbours.size(); i++){
            edge e = neighbours[i];
            int neighbour = e.first;
            double neighbourD = e.second;
            
            double newD = curD + neighbourD;
            //printf("Trying neighbour %i with distance %F\n", neighbour, d);
            if (newD < SUMLO && newD < distance[neighbour]){ // cut off any edges more than SUMLO
                Q.erase(make_pair(distance[neighbour], neighbour));
                //if (neighbourD == 0)
                //    distance[neighbour] = 0; // 0-edges reset the Dijkstra's
                //else 
                    distance[neighbour] = curD + neighbourD;
                Q.insert(make_pair(distance[neighbour], neighbour));
            }       
        }
    }   
    // TODO deal with SumLO and with 0-edges RESETTING the Dijstra's.
    
     // create object array
    for (int i = 0; i< NUM_PIXELS; i++){
        object[i] = distance[i] < SUMLO;
    }
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
    if ( nrhs != 2 )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","Must have two inputs");
    if ( mxGetClassID(prhs[0]) != mxDOUBLE_CLASS )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","image must be of type double");
    if ( mxGetClassID(prhs[1]) != mxSTRUCT_CLASS )
        mexErrMsgIdAndTxt("regiongrow_wraper:main","parameters argument must be a structure");
    
    
    /* first argument - the (2D) image */
    double * pixels = (double*)mxGetData(prhs[0]);
    const int* image_dims = mxGetDimensions(prhs[0]);
    int image_ndim = mxGetNumberOfDimensions(prhs[0]);
    W = image_dims[1];
    H = image_dims[0];
    NUM_PIXELS = W * H;
    
    
    /* second input - parameters structure */    

    GetScalar(mxGetField(prhs[1], 0, "hi"), HI);
    GetScalar(mxGetField(prhs[1], 0, "lo"), LO);
    GetScalar(mxGetField(prhs[1], 0, "sumlo"), SUMLO);
    
    
/*
    
    // 1. transform image into a 2D grid
    double image[H][W]; 
    for (int i = 0; i < H; i++){
        for (int j = 0; j < W; j++){
            image[i][j] = pixels[i*W + j];
        }
    }
    
    // 2. flatten back
    double image2[W*H];
    for (int i = 0; i < H ; i++){
        for (int j = 0; j < W; j++){
            image2[i*W + j] = image[i][j];
        }
    }
    */
    
    
    // first output - the feature space raw image
    plhs[0] = mxCreateNumericArray(image_ndim, image_dims, mxLOGICAL_CLASS, mxREAL);
    bool* outImage = (bool*)mxGetData(plhs[0]);
    for (int i = 0; i < NUM_PIXELS; i++){
        outImage[i] = false;
    }
    
    
  

    adj A = buildAdj(pixels);
    vector<int> seeds;
    int middlePixel = toLinear(H/2, W/2);
    seeds.push_back(middlePixel);
    
    dijkstra(A, seeds, outImage);
    
    
    
/*
    mxArray * mxconf = NULL;
    float * conf = NULL;
    mxArray * mxgrad = NULL;
    float * grad = NULL;
    mxArray * mxwght = NULL;
    float * wght = NULL;
    
    if (syn) {
        // perform synergistic segmentation 
        int maps_dim[2] = {w*h, 1};
        // allcate memory for confidence and gradient maps 
        mxconf = mxCreateNumericArray(2, maps_dim, mxSINGLE_CLASS, mxREAL);
        conf = (float*)mxGetData(mxconf);
        mxgrad = mxCreateNumericArray(2, maps_dim, mxSINGLE_CLASS, mxREAL);
        grad = (float*)mxGetData(mxgrad);
        
        BgImage rgbIm;
        rgbIm.SetImage(rgbim, w, h, rgb_dims[0] == 3);
        BgEdgeDetect edgeDetector(grWin);
        edgeDetector.ComputeEdgeInfo(&rgbIm, conf, grad);
        
        mxwght = mxCreateNumericArray(2, maps_dim, mxSINGLE_CLASS, mxREAL);
        wght = (float*)mxGetData(mxgrad);
        
        for ( ii = 0 ; ii < w*h; ii++ ) {
            wght[ii] = (grad[ii] > .002) ? aij*grad[ii]+(1-aij)*conf[ii] : 0;
        }
        ms.SetWeightMap(wght, edgeThr);
        if (ms.ErrorStatus)
            mexErrMsgIdAndTxt("edison_wraper:edison","Mean shift set weights: %s", ms.ErrorMessage);

    }
    ms.Filter(spBW, fsBW, sul);
    if (ms.ErrorStatus)
        mexErrMsgIdAndTxt("edison_wraper:edison","Mean shift filter: %s", ms.ErrorMessage);

    if (steps == 2) {
        ms.FuseRegions(fsBW, minArea);
        if (ms.ErrorStatus)
            mexErrMsgIdAndTxt("edison_wraper:edison","Mean shift fuse: %s", ms.ErrorMessage);
    }
    
    if ( nlhs >= 1 ) {
        // first output - the feture space raw image
        plhs[0] = mxCreateNumericArray(image_ndim, image_dims, mxSINGLE_CLASS, mxREAL);
        fimage = (float*)mxGetData(plhs[0]);
        ms.GetRawData(fimage);
    }
    
    int* labels;
    float* modes;
    int* count;
    int RegionCount = ms.GetRegions(&labels, &modes, &count);

    if ( nlhs >= 2 ) {
        // second output - labeled image
        plhs[1] = mxCreateNumericArray(2, image_dims+1, mxINT32_CLASS, mxREAL);
        int* plabels = (int*)mxGetData(plhs[1]);
        for (ii=0; ii< w*h; ii++)
            plabels[ii] = labels[ii];
    }
    delete [] labels;
    int arr_dims[2];
    if ( nlhs >= 3 ) {
        // third output - the modes
        arr_dims[0] = N;
        arr_dims[1] = RegionCount;
        plhs[2] = mxCreateNumericArray(2, arr_dims, mxSINGLE_CLASS, mxREAL);
        fimage = (float*)mxGetData(plhs[2]);
        for (ii=0;ii<N*RegionCount; ii++)
            fimage[ii] = modes[ii];
    }
    delete [] modes;
    
    if ( nlhs >= 4 ) {
        // fourth output - region sizes (# of pixels)
        arr_dims[0] = 1;
        arr_dims[1] = RegionCount;
        plhs[3] = mxCreateNumericArray(2, arr_dims, mxINT32_CLASS, mxREAL);
        int * pc = (int*)mxGetData(plhs[3]);
        for (ii=0;ii<RegionCount; ii++)
            pc[ii] = count[ii];
    }   
    delete [] count;
    
    if ( !syn )
        return;
    
    if ( nlhs >= 5)
        // fifth output - gradient map
        plhs[4] = mxgrad;
    else
        mxDestroyArray(mxgrad);
    
    if ( nlhs >= 6)
        plhs[5] = mxconf;
    else
        mxDestroyArray(mxconf);
    
*/  
   
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

