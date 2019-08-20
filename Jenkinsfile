openriscPipeline {
    job('verilator') {
        job 'verilator'
    }

    job('icarus') {
        job 'or1k-tests'
        sim 'icarus'
        expectedFailures 'or1k-cy or1k-ov or1k-shortjump'
    }
}
