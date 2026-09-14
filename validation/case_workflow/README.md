# Case workflow validation

23 tests passed. The real Cycles synthetic fixture built and passed mesh audit, paused at review, resumed to freeze with an integration_test approval, and reused the frozen output on the next run. A code change invalidated the previous attempts and approval. No real scene was approved and no hardware was used. Intake missing-data behavior, output tampering, failed-stage retries, template materialization and path/dependency checks are covered by tests.
