#include <tools.h>
#include <timer.h>
#include <ts7200.h>


// TODO: integrate clock and clock_server if necessary
void start_clock( int load_val ) {
	// load a value to TIMER3_BASE
	int *timer_base = (int *)TIMER3_BASE;
	int *clk_ldr = timer_base + (LDR_OFFSET / 4);
	int *clk_ctrl = timer_base + (CRTL_OFFSET / 4);

	*clk_ldr = load_val;
	// Enable, set mode, and use 508MHz
	*clk_ctrl = *clk_ctrl | ENABLE_MASK | MODE_MASK | CLKSEL_MASK;
}

// Get value from value register
int get_timer_val( ) {
	int *timer_base = (int *)TIMER3_BASE;
	int *clk_val = timer_base + (VAL_OFFSET / 4);
	return *clk_val;
}

