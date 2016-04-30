class simple_point{
	public:
		double x;
		double y;

		simple_point(double xin, double yin){
			x = xin;
			y = yin;
		}
		simple_point(){}
		static double eucDist(simple_point & a, simple_point & b){
			return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
		}
};
