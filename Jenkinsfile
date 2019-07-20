pipeline {
    agent any

    stages {
        stage("Docker pull") {
            steps {
                sh 'docker pull librecores/librecores-ci-openrisc'
                sh 'docker images'
            }
        }

        stage("Docker run") {
            parallel {
                stage("verilator") {
                    environment {
                        JOB = 'verilator'
                    }
                    steps {
                        dockerrun()
                    }
                }
                stage("testing") {
                    environment {
                        JOB = 'or1k-tests'
                        SIM = 'icarus'
                        EXPECTED_FAILURES = "or1k-cy or1k-ov or1k-shortjump"
                    }
                    steps {
                        dockerrun()
                    }
                }
            }
        }
    }
}

void dockerrun() {
    sh 'docker run --rm -v $(pwd):/src -e "JOB=$JOB" -e "SIM=$SIM" -e "EXPECTED_FAILURES=$EXPECTED_FAILURES" librecores/librecores-ci-openrisc /src/.travis/test.sh'
}