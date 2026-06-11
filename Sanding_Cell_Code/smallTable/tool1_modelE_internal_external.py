from smallTable.tool1_modelD_internal import (
    smalldoor1tool3 as smalldoor1tool3_internal,
    smalldoor2tool3 as smalldoor2tool3_internal,
    smalldoor3tool3 as smalldoor3tool3_internal,
    smalldoor4tool3 as smalldoor4tool3_internal,
)
from smallTable.frame1tool3 import (
    smalldoor1tool3 as smalldoor1tool3_external,
    smalldoor2tool3 as smalldoor2tool3_external,
    smalldoor3tool3 as smalldoor3tool3_external,
    smalldoor4tool3 as smalldoor4tool3_external,
)


def _run_external_then_internal(external_fn, internal_fn, z, cps, force=None):
    # Layer 1: external side (Model C logic).
    # frame1tool3 returns through raised prehoming points after outer execution.
    external_fn(z=z, cps=cps, force=force)

    # Layer 2: internal pocket side (Model D logic).
    internal_fn(z=z, cps=cps, force=force)


def smalldoor1tool3(z, cps, force=None):
    _run_external_then_internal(
        smalldoor1tool3_external, smalldoor1tool3_internal, z, cps, force=force
    )


def smalldoor2tool3(z, cps, force=None):
    _run_external_then_internal(
        smalldoor2tool3_external, smalldoor2tool3_internal, z, cps, force=force
    )


def smalldoor3tool3(z, cps, force=None):
    _run_external_then_internal(
        smalldoor3tool3_external, smalldoor3tool3_internal, z, cps, force=force
    )


def smalldoor4tool3(z, cps, force=None):
    _run_external_then_internal(
        smalldoor4tool3_external, smalldoor4tool3_internal, z, cps, force=force
    )
