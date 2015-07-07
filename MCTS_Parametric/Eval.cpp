#include "Eval.h"
#include <stack>
#include <boost/spirit.hpp>
#include <iostream>

using namespace std;
using namespace boost::spirit;

namespace eval {

stack<double> stk; //stack<int> stk;

void push(double n) //push(int n)
{
	stk.push(n);
}

double pop()  //int pop()
{
	double t = stk.top(); //int t = stk.top();
	stk.pop();
	return t;
}

void clear()
{
	stk = stack<double>();  //stk = stack<int>();
}

//-----------------------------------------------------------------------------
//	セマンティックアクション
//-----------------------------------------------------------------------------

void PUSH(double n) //PUSH(int n)
{
	stk.push( n );
}

void ADD(const char* const, const char* const)
{
	double a = pop();//int a = pop();
	double b = pop();//int b = pop();
	push(a + b);
}

void SUB(const char* const, const char* const)
{
	double a = pop();//int a = pop();
	double b = pop();//int b = pop();
	push(b - a);
}

void MUL(const char* const, const char* const)
{
	double a = pop();//int a = pop();
	double b = pop();//int b = pop();  
	push(a * b);
}

void DIV(const char* const, const char* const)
{
	double a = pop();//int a = pop();
	double b = pop();//int b = pop();
	push(b / a);
}

void POW(const char* const, const char* const)
{
	double a = pop();//int a = pop();
	double b = pop();//int b = pop();
	push(pow(b,a));
}

void GREATER(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b > a ? 1.0 : 0.0);
}

void LESS(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b < a ? 1.0 : 0.0);
}

void EQUAL(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b == a ? 1.0 : 0.0);
}

void GREATER_EQUAL(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b >= a ? 1.0 : 0.0);
}

void LESS_EQUAL(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b <= a ? 1.0 : 0.0);
}

void NOT_EQUAL(const char* const, const char* const) {
	double a = pop();
	double b = pop();
	push(b != a ? 1.0 : 0.0);
}

//-----------------------------------------------------------------------------
//	文法
//-----------------------------------------------------------------------------
struct compare_grammar : public grammar<compare_grammar> {
	template<typename S> struct definition {
		rule<S> comparison, expression, term, factor, group, comparison_op;

		definition(const compare_grammar& self) {
			group		= '(' >> expression >> ')';
			factor		= real_p[&PUSH] | group;
			term		= factor >> *( ('*' >> factor)[&MUL] |
				                       ('/' >> factor)[&DIV] |
									   ('^' >> factor)[&POW]   );
			expression	= term >> *( ('+' >> term)[&ADD] | ('-' >> term)[&SUB] );
			comparison = expression >> ( ('>' >> expression[&GREATER]) |
										 ('<' >> expression[&LESS]) |
										 ('=' >> expression[&EQUAL]) |
										 (">=" >> expression[&GREATER_EQUAL]) |
										 ("<=" >> expression[&LESS_EQUAL]) |
										 ("!=" >> expression[&NOT_EQUAL]) );
		}

		// 開始記号を定義
		const rule<S>& start() const { return comparison; }	
	};
};

struct calculate_grammar : public grammar<calculate_grammar> {
	template<typename S> struct definition {
		rule<S> expression, term, factor, group;

		definition(const calculate_grammar& self) {
			group		= '(' >> expression >> ')';
			factor		= real_p[&PUSH] | group;
			term		= factor >> *( ('*' >> factor)[&MUL] |
				                       ('/' >> factor)[&DIV] |
									   ('^' >> factor)[&POW]   );
			expression	= term >> *( ('+' >> term)[&ADD] | ('-' >> term)[&SUB] );
		}

		// 開始記号を定義
		const rule<S>& start() const { return expression; }	
	};
};

bool compare(string str) {
	compare_grammar grammar;

	// 入力された文字列をcalcに入力し、space_p(空白、タブ、改行)を抜いて解析
	parse_info<> r = parse(str.c_str(), grammar, space_p);

	if (r.full) {
		return pop() == 1.0 ? true : false;
	} else {
		cout << "error" << endl;
		return false;
	}
}

double calculate(string str) {
	calculate_grammar grammar;

	// 入力された文字列をcalcに入力し、space_p(空白、タブ、改行)を抜いて解析
	parse_info<> r = parse(str.c_str(), grammar, space_p);

	if (r.full) {
		return pop();
	} else {
		cout << "error" << endl;
		return 0;
	}
}

}
