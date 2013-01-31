 // The following code inverts the matrix input using LU-decomposition
 // with backsubstitution of unit vectors.
 // Reference: Numerical Recipies in C, 2nd ed., by Press, Teukolsky, Vetterling & Flannery.
//
// http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?action=browse&diff=1&id=LU_Matrix_Inversion
 // Hope someone finds this useful. Regards, Fredrik Orderud. 
//
 // Last edited September 4, 2007 5:23 am
 //
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>



namespace ublas = boost::numeric::ublas;

// Matrix inversion routine.
// Uses lu_factorize and lu_substitute in uBLAS to invert a matrix
template<class T>
bool invert (const ublas::matrix<T>& input, ublas::matrix<T>& inverse) {
	using namespace boost::numeric::ublas;
	typedef permutation_matrix<std::size_t> pmatrix;
	// create a working copy of the input
	matrix<T> A(input);
	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A,pm);
	if( res != 0 ) return false;

	// create identity matrix of "inverse"
	inverse.assign(ublas::identity_matrix<T>(A.size1()));
 
	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);
 
	return true;
 }

template<class T>
boost::numeric::ublas::matrix<T>
invert(const boost::numeric::ublas::matrix<T> &m, bool &is_singular)
{ 
	ublas::matrix<T> inv_m (m.size1(), m.size2());
	is_singular = ! invert (m, inv_m);
	return inv_m;
}


// http://archives.free.net.ph/message/20080909.064313.59c122c4.fr.html 
template<class matrix_T>
double determinant(ublas::matrix_expression<matrix_T> const& mat_r) {
	double det = 1.0;
	matrix_T mLu(mat_r());
	ublas::permutation_matrix<std::size_t> pivots(mat_r().size1() );
	bool is_singular = lu_factorize(mLu, pivots);
	if (!is_singular) {
		for (std::size_t i = 0; i < pivots.size(); ++i) {
			if (pivots(i) != i) {
			det *= -1.0;
		}
		det *= mLu(i,i);
     }
	} else {
		det = 0.0;
	}
	return det;
} 

inline int signof(double a) { return (a == 0) ? 0 : (a<0 ? -1 : 1); }

inline double pi_to_pi(double angle){
	double pi = 3.14159265359;
         if ((angle <= -2*pi)||(angle >= 2*pi)) {angle =  fmod (angle,2*pi);}
         if (angle >  pi) {angle = angle-2*pi;}
         if (angle < -pi) {angle = angle+2*pi;}
         return angle;
}

inline double SampleNormal (double mean, double sigma)
{
    using namespace boost;
	// Create a Mersenne twister random number generator
    // that is seeded once with #seconds since 1970
    static mt19937 rng(static_cast<unsigned> (std::time(0)));
 
    // select Gaussian probability distribution
    normal_distribution<double> norm_dist(mean, sigma);
 
    // bind random number generator to distribution, forming a function
    variate_generator<mt19937&, normal_distribution<double> >  normal_sampler(rng, norm_dist);
 
    // sample from the distribution
    return normal_sampler();
}
    

double uniform()
{
        using namespace boost;
		static mt19937 rng(static_cast<unsigned> (std::time(0)));
		uniform_real<> uniform(0.0, 1.0);
        variate_generator< mt19937&, uniform_real<> > uniform_sampler(rng, uniform);
        return uniform_sampler();
}

ublas::vector<double> uniform_vector(unsigned size)
{       
		ublas::vector<double> r(size);
        for (unsigned i = 0; i < size; ++i)
            r(i) = uniform();     
        return r;
}
  