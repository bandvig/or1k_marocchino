openriscPipeline {

    yosysReport {
		core 'or1k_marocchino'
		target 'synth'
		logPath 'build/or1k_marocchino_*/synth-icestorm/yosys.log'
	}

    job('verilator') {
        job 'verilator'
    }

    job('icarus') {
        job 'or1k-tests'
        sim 'icarus'
        expectedFailures 'or1k-cy or1k-ov or1k-shortjump'
    }
}
