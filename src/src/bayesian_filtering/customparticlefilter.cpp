#include "bayesian_filtering/customparticlefilter.h"

using namespace MatrixWrapper;
using namespace BFL;

CustomParticleFilter::CustomParticleFilter(MCPdf<ColumnVector> *prior, int resampleperiod, double resamplethreshold, int resamplescheme):
    BootstrapFilter<ColumnVector,ColumnVector>(prior, resampleperiod, resamplethreshold, resamplescheme)
{ }

vector<WeightedSample<ColumnVector> > CustomParticleFilter::getNewSamples()
{
    return _new_samples;
}
